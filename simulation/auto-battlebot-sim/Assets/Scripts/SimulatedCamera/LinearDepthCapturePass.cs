// Auto-Battlebot Simulation System
// LinearDepthCapturePass - Depth camera capture for TCP transfer (SIM-007)
//
// HDRP Custom Pass that captures the depth buffer and linearizes it
// using a compute shader. Outputs to a RenderTexture for AsyncGPUReadback
// and TCP transfer to the C++ application.

using System;
using System.Diagnostics;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using Debug = UnityEngine.Debug;

namespace AutoBattlebot.SimulatedCamera
{
    /// <summary>
    /// HDRP Custom Pass that captures linearized depth for AsyncGPUReadback and TCP transfer.
    /// 
    /// This pass:
    /// - Runs after opaque depth rendering (AfterOpaqueDepthAndNormal)
    /// - Copies and linearizes the depth buffer using a compute shader
    /// - Outputs to a R32_SFloat RenderTexture (float meters)
    /// - Optionally adds sensor noise for realism
    /// - Compatible with AsyncGPUReadback for TCP transfer
    /// 
    /// Depth output characteristics (matching ZED 2i):
    /// - Values are in meters (linearized)
    /// - Valid range: 0.3m - 20m
    /// - Invalid depth (sky, out of range): 0.0 or configurable
    /// - Format: R32_SFloat (32-bit float per pixel)
    /// 
    /// Usage:
    /// 1. Add a Custom Pass Volume to the scene
    /// 2. Add this pass to the volume
    /// 3. Assign the compute shader
    /// 4. CommunicationBridge will use AsyncGPUReadback on DepthTexture
    /// </summary>
    public class LinearDepthCapturePass : CustomPass
    {
        #region Serialized Fields

        [Header("Output Configuration")]
        [Tooltip("Width of the output depth texture (should match RGB camera)")]
        public int width = 1280;

        [Tooltip("Height of the output depth texture (should match RGB camera)")]
        public int height = 720;

        [Header("Depth Range (ZED 2i defaults)")]
        [Tooltip("Near clip plane in meters (ZED 2i min: 0.3m)")]
        public float nearClip = 0.3f;

        [Tooltip("Far clip plane in meters (ZED 2i max: 20m)")]
        public float farClip = 20.0f;

        [Tooltip("Value to use for invalid/sky depth (0 = no depth)")]
        public float invalidDepthValue = 0.0f;

        [Header("Shader Configuration")]
        [Tooltip("Use blit shader instead of compute shader (more HDRP compatible)")]
        public bool useBlitShader = true;

        [Tooltip("Blit shader for depth linearization (HDRP compatible)")]
        public Shader linearizeDepthBlitShader;

        [Tooltip("Compute shader for depth linearization (alternative)")]
        public ComputeShader linearizeDepthShader;

        [Header("Depth Noise (Optional)")]
        [Tooltip("Enable depth noise simulation for realism")]
        public bool enableNoise = false;

        [Tooltip("Base noise level (meters) - adds uniform noise")]
        [Range(0f, 0.01f)]
        public float baseNoiseLevel = 0.002f;

        [Tooltip("Distance-dependent noise factor (noise increases with depth)")]
        [Range(0f, 0.01f)]
        public float distanceNoiseFactor = 0.001f;

        [Header("Camera Filtering")]
        [Tooltip("Target camera for depth capture (required - only this camera's depth will be captured)")]
        public Camera targetCamera;

        [Header("Debug Options")]
        [Tooltip("Enable CPU readback for debugging (slower)")]
        public bool enableCpuReadback = false;

        [Tooltip("Enable performance profiling")]
        public bool enableProfiling = false;

        [Tooltip("Debug mode: 0=normal, 1=constant 5m, 2=UV gradient, 3=raw depth")]
        [Range(0, 3)]
        public int debugMode = 0;

        #endregion

        #region Private Fields

        private RenderTexture _depthTexture;
        private Texture2D _cpuTexture;
        private int _kernelId = -1;
        private int _kernelIdNoisy = -1;
        private bool _isInitialized = false;
        private Material _blitMaterial;

        // Shader property IDs (cached for performance)
        private static readonly int _DepthInputId = Shader.PropertyToID("_DepthInput");
        private static readonly int _LinearDepthOutputId = Shader.PropertyToID("_LinearDepthOutput");
        private static readonly int _ZBufferParamsId = Shader.PropertyToID("_ZBufferParams");
        private static readonly int _DepthRangeId = Shader.PropertyToID("_DepthRange");
        private static readonly int _InvalidDepthId = Shader.PropertyToID("_InvalidDepth");
        private static readonly int _TextureSizeId = Shader.PropertyToID("_TextureSize");
        private static readonly int _NoiseParamsId = Shader.PropertyToID("_NoiseParams");
        private static readonly int _TimeId = Shader.PropertyToID("_Time");
        private static readonly int _DebugModeId = Shader.PropertyToID("_DebugMode");
        private static readonly int _DepthBufferSizeId = Shader.PropertyToID("_DepthBufferSize");

        // Profiling
        private Stopwatch _profilerStopwatch = new Stopwatch();
        private long _totalExecuteTimeUs = 0;
        private int _executeCount = 0;

        #endregion

        #region Properties

        /// <summary>
        /// The linearized depth RenderTexture for AsyncGPUReadback.
        /// Format: R32_SFloat (single-channel float, values in meters)
        /// </summary>
        public RenderTexture DepthTexture => _depthTexture;

        /// <summary>
        /// Whether the pass is initialized.
        /// </summary>
        public bool IsInitialized => _isInitialized;

        /// <summary>
        /// Whether the depth texture has been created.
        /// </summary>
        public bool HasTexture => _depthTexture != null && _depthTexture.IsCreated();

        /// <summary>
        /// Native texture pointer (for debugging/logging).
        /// </summary>
        public IntPtr NativeTexturePtr => HasTexture ? _depthTexture.GetNativeTexturePtr() : IntPtr.Zero;

        /// <summary>
        /// Average execute time in microseconds.
        /// </summary>
        public double AverageExecuteTimeUs => _executeCount > 0 ? (double)_totalExecuteTimeUs / _executeCount : 0;

        #endregion

        #region CustomPass Overrides

        protected override void Setup(ScriptableRenderContext renderContext, CommandBuffer cmd)
        {
            name = "Linear Depth Capture";

            if (useBlitShader)
            {
                // Use blit shader approach (more HDRP compatible)
                if (linearizeDepthBlitShader == null)
                {
                    // Try to find the shader by name
                    linearizeDepthBlitShader = Shader.Find("AutoBattlebot/LinearizeDepthBlit");
                }

                if (linearizeDepthBlitShader == null)
                {
                    Debug.LogError("[LinearDepthCapturePass] Blit shader not assigned and not found");
                    return;
                }

                _blitMaterial = CoreUtils.CreateEngineMaterial(linearizeDepthBlitShader);
                if (_blitMaterial == null)
                {
                    Debug.LogError("[LinearDepthCapturePass] Failed to create blit material");
                    return;
                }
            }
            else
            {
                // Use compute shader approach
                if (linearizeDepthShader == null)
                {
                    Debug.LogError("[LinearDepthCapturePass] Compute shader not assigned");
                    return;
                }

                // Find kernels
                _kernelId = linearizeDepthShader.FindKernel("LinearizeDepth");
                if (_kernelId < 0)
                {
                    Debug.LogError("[LinearDepthCapturePass] Kernel 'LinearizeDepth' not found in compute shader");
                    return;
                }

                // Try to find noisy kernel (optional)
                if (linearizeDepthShader.HasKernel("LinearizeDepthNoisy"))
                {
                    _kernelIdNoisy = linearizeDepthShader.FindKernel("LinearizeDepthNoisy");
                }
            }

            // Create output texture
            CreateDepthTexture();

            _isInitialized = true;
            Debug.Log($"[LinearDepthCapturePass] Setup complete: {width}x{height}, " +
                      $"Range={nearClip}m-{farClip}m, " +
                      $"Noise={enableNoise}, " +
                      $"UseBlitShader={useBlitShader}, " +
                      $"Format={_depthTexture.graphicsFormat}");
        }

        protected override void Execute(CustomPassContext ctx)
        {
            if (!_isInitialized)
            {
                return;
            }

            // CRITICAL: Only execute for the target camera
            // This prevents capturing depth from the wrong camera (e.g., player camera)
            if (targetCamera != null && ctx.hdCamera.camera != targetCamera)
            {
                return;
            }

            if (useBlitShader && _blitMaterial == null)
            {
                return;
            }

            if (!useBlitShader && _kernelId < 0)
            {
                return;
            }

            if (enableProfiling)
            {
                _profilerStopwatch.Restart();
            }

            // Ensure texture exists and matches size
            if (!HasTexture || _depthTexture.width != width || _depthTexture.height != height)
            {
                CreateDepthTexture();
            }

            // Debug logging (first frame only)
            if (_executeCount == 0)
            {
                var camera = ctx.hdCamera.camera;
                int depthBufferWidth = ctx.hdCamera.actualWidth;
                int depthBufferHeight = ctx.hdCamera.actualHeight;
                
                Debug.Log($"[LinearDepthCapturePass] Execute first frame: " +
                          $"camera={camera.name}, " +
                          $"targetCamera={(targetCamera != null ? targetCamera.name : "ANY")}, " +
                          $"output={_depthTexture.width}x{_depthTexture.height}, " +
                          $"depthBuffer={depthBufferWidth}x{depthBufferHeight}, " +
                          $"useBlitShader={useBlitShader}, " +
                          $"nearClip={nearClip}, farClip={farClip}");
                
                // Warn if there's a resolution mismatch that was previously causing issues
                if (depthBufferWidth != _depthTexture.width || depthBufferHeight != _depthTexture.height)
                {
                    Debug.LogWarning($"[LinearDepthCapturePass] Resolution mismatch detected! " +
                                   $"Output: {_depthTexture.width}x{_depthTexture.height}, " +
                                   $"DepthBuffer: {depthBufferWidth}x{depthBufferHeight}. " +
                                   $"This is handled by the shader, but may indicate misconfiguration.");
                }
            }

            if (useBlitShader)
            {
                ExecuteBlitShader(ctx);
            }
            else
            {
                ExecuteComputeShader(ctx);
            }

            if (enableProfiling)
            {
                _profilerStopwatch.Stop();
                _totalExecuteTimeUs += _profilerStopwatch.ElapsedTicks * 1000000 / Stopwatch.Frequency;
            }
            _executeCount++;
        }

        private void ExecuteBlitShader(CustomPassContext ctx)
        {
            // Get the actual depth buffer dimensions from HDRP
            // This is critical - the depth buffer may have different dimensions than the output texture
            // due to HDRP's internal rendering resolution, dynamic resolution, or render scale settings
            int depthBufferWidth = ctx.hdCamera.actualWidth;
            int depthBufferHeight = ctx.hdCamera.actualHeight;

            // Set material parameters
            _blitMaterial.SetVector(_DepthRangeId, new Vector4(nearClip, farClip, 0, 0));
            _blitMaterial.SetFloat(_InvalidDepthId, invalidDepthValue);
            _blitMaterial.SetFloat(_DebugModeId, debugMode);
            
            // Pass the actual depth buffer size to handle resolution mismatches
            _blitMaterial.SetVector(_DepthBufferSizeId, new Vector4(
                depthBufferWidth, 
                depthBufferHeight, 
                1.0f / depthBufferWidth, 
                1.0f / depthBufferHeight));

            // Debug logging on first frame
            if (_executeCount == 0)
            {
                Debug.Log($"[LinearDepthCapturePass] ExecuteBlitShader: debugMode={debugMode}, " +
                          $"depthRange=({nearClip}, {farClip}), " +
                          $"renderTarget={_depthTexture.width}x{_depthTexture.height}, " +
                          $"depthBuffer={depthBufferWidth}x{depthBufferHeight}");
            }

            // Blit using HDRP's full-screen triangle
            // SetRenderTarget and draw full-screen triangle
            ctx.cmd.SetRenderTarget(_depthTexture);
            ctx.cmd.DrawProcedural(Matrix4x4.identity, _blitMaterial, 0, MeshTopology.Triangles, 3, 1);
        }

        private void ExecuteComputeShader(CustomPassContext ctx)
        {
            // Get ZBufferParams from the camera
            var camera = ctx.hdCamera.camera;
            Vector4 zBufferParams = CalculateZBufferParams(camera);

            // Get the actual depth buffer dimensions from HDRP
            // This is critical - the depth buffer may have different dimensions than the output texture
            int depthBufferWidth = ctx.hdCamera.actualWidth;
            int depthBufferHeight = ctx.hdCamera.actualHeight;

            // Select kernel based on noise setting
            int kernelToUse = (enableNoise && _kernelIdNoisy >= 0) ? _kernelIdNoisy : _kernelId;

            // Set compute shader parameters
            // HDRP's cameraDepthBuffer is a Texture2DArray, we sample slice 0 (main view)
            int sliceIndex = 0;
            ctx.cmd.SetComputeTextureParam(linearizeDepthShader, kernelToUse, _DepthInputId, ctx.cameraDepthBuffer);
            ctx.cmd.SetComputeTextureParam(linearizeDepthShader, kernelToUse, _LinearDepthOutputId, _depthTexture);
            ctx.cmd.SetComputeVectorParam(linearizeDepthShader, _ZBufferParamsId, zBufferParams);
            ctx.cmd.SetComputeVectorParam(linearizeDepthShader, _DepthRangeId, new Vector4(nearClip, farClip, 0, 0));
            ctx.cmd.SetComputeFloatParam(linearizeDepthShader, _InvalidDepthId, invalidDepthValue);
            ctx.cmd.SetComputeVectorParam(linearizeDepthShader, _TextureSizeId, new Vector4(width, height, sliceIndex, 0));
            ctx.cmd.SetComputeVectorParam(linearizeDepthShader, _DepthBufferSizeId, new Vector4(depthBufferWidth, depthBufferHeight, 0, 0));

            // Set noise parameters if using noisy kernel
            if (enableNoise && _kernelIdNoisy >= 0)
            {
                ctx.cmd.SetComputeVectorParam(linearizeDepthShader, _NoiseParamsId, 
                    new Vector4(baseNoiseLevel, distanceNoiseFactor, 0, 0));
                ctx.cmd.SetComputeFloatParam(linearizeDepthShader, _TimeId, Time.time);
            }

            // Dispatch compute shader
            // Thread group size is 8x8, so we need (width/8) x (height/8) groups
            int groupsX = Mathf.CeilToInt(width / 8.0f);
            int groupsY = Mathf.CeilToInt(height / 8.0f);

            ctx.cmd.DispatchCompute(linearizeDepthShader, kernelToUse, groupsX, groupsY, 1);
        }

        protected override void Cleanup()
        {
            if (_cpuTexture != null)
            {
                CoreUtils.Destroy(_cpuTexture);
                _cpuTexture = null;
            }

            if (_depthTexture != null)
            {
                _depthTexture.Release();
                CoreUtils.Destroy(_depthTexture);
                _depthTexture = null;
            }

            if (_blitMaterial != null)
            {
                CoreUtils.Destroy(_blitMaterial);
                _blitMaterial = null;
            }

            _isInitialized = false;
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Resize the depth texture.
        /// </summary>
        public void Resize(int newWidth, int newHeight)
        {
            width = newWidth;
            height = newHeight;
            CreateDepthTexture();
        }

        /// <summary>
        /// Read depth to CPU (for debugging or data generation).
        /// Only works if enableCpuReadback is true.
        /// Warning: This is slow and blocks. Use AsyncGPUReadback for production.
        /// </summary>
        public Texture2D ReadToCpu()
        {
            if (!enableCpuReadback)
            {
                Debug.LogWarning("[LinearDepthCapturePass] CPU readback is disabled. Use AsyncGPUReadback for production.");
                return null;
            }

            if (!HasTexture)
            {
                return null;
            }

            // Create CPU texture if needed
            if (_cpuTexture == null || _cpuTexture.width != width || _cpuTexture.height != height)
            {
                if (_cpuTexture != null)
                {
                    CoreUtils.Destroy(_cpuTexture);
                }
                _cpuTexture = new Texture2D(width, height, TextureFormat.RFloat, false);
            }

            // Read from GPU to CPU
            var prevActive = RenderTexture.active;
            RenderTexture.active = _depthTexture;
            _cpuTexture.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            _cpuTexture.Apply();
            RenderTexture.active = prevActive;

            return _cpuTexture;
        }

        /// <summary>
        /// Get a performance report string.
        /// </summary>
        public string GetPerformanceReport()
        {
            return $"[LinearDepthCapturePass] Performance:\n" +
                   $"  Resolution: {width}x{height}\n" +
                   $"  Executions: {_executeCount}\n" +
                   $"  Avg Execute Time: {AverageExecuteTimeUs:F1}µs";
        }

        /// <summary>
        /// Reset performance statistics.
        /// </summary>
        public void ResetProfilingStats()
        {
            _totalExecuteTimeUs = 0;
            _executeCount = 0;
        }

        #endregion

        #region Private Methods

        private void CreateDepthTexture()
        {
            // Clean up existing texture
            if (_depthTexture != null)
            {
                _depthTexture.Release();
                CoreUtils.Destroy(_depthTexture);
            }

            // Create AsyncGPUReadback-compatible depth texture
            // Use RenderTextureDescriptor for explicit format control
            var desc = new RenderTextureDescriptor(width, height)
            {
                graphicsFormat = UnityEngine.Experimental.Rendering.GraphicsFormat.R32_SFloat,
                depthBufferBits = 0,
                msaaSamples = 1,
                useMipMap = false,
                autoGenerateMips = false,
                enableRandomWrite = true,  // Required for compute shader output
                dimension = TextureDimension.Tex2D,
                volumeDepth = 1
            };

            _depthTexture = new RenderTexture(desc)
            {
                filterMode = FilterMode.Point,  // No filtering for depth
                wrapMode = TextureWrapMode.Clamp,
                name = $"LinearDepthCapture_{width}x{height}"
            };

            if (!_depthTexture.Create())
            {
                Debug.LogError("[LinearDepthCapturePass] Failed to create depth RenderTexture");
            }
        }

        private Vector4 CalculateZBufferParams(Camera camera)
        {
            // Unity's _ZBufferParams calculation for reversed Z-buffer
            // x = 1 - far/near
            // y = far/near
            // z = x/far
            // w = y/far
            float near = camera.nearClipPlane;
            float far = camera.farClipPlane;

            float x = 1.0f - far / near;
            float y = far / near;
            float z = x / far;
            float w = y / far;

            return new Vector4(x, y, z, w);
        }

        #endregion
    }
}
