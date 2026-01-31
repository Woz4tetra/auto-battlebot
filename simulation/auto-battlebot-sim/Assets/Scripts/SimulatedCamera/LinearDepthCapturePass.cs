// Auto-Battlebot Simulation System
// LinearDepthCapturePass - Depth camera capture for CUDA Interop (SIM-007)
//
// HDRP Custom Pass that captures the depth buffer and linearizes it
// using a compute shader. Outputs to a RenderTexture for CUDA Interop.

using System;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace AutoBattlebot.SimulatedCamera
{
    /// <summary>
    /// HDRP Custom Pass that captures linearized depth for CUDA Interop.
    /// 
    /// This pass:
    /// - Runs after opaque depth rendering (AfterOpaqueDepthAndNormal)
    /// - Copies and linearizes the depth buffer using a compute shader
    /// - Outputs to a R32_SFloat RenderTexture (float meters)
    /// - Exposes native texture pointer for CUDA registration
    /// 
    /// Usage:
    /// 1. Add a Custom Pass Volume to the scene
    /// 2. Add this pass to the volume
    /// 3. Assign the compute shader
    /// 4. Access DepthTexture.GetNativeTexturePtr() for CUDA registration
    /// </summary>
    public class LinearDepthCapturePass : CustomPass
    {
        #region Serialized Fields

        [Header("Output Configuration")]
        [Tooltip("Width of the output depth texture (should match RGB camera)")]
        public int width = 1280;

        [Tooltip("Height of the output depth texture (should match RGB camera)")]
        public int height = 720;

        [Header("Depth Range")]
        [Tooltip("Near clip plane in meters (values closer are clamped)")]
        public float nearClip = 0.3f;

        [Tooltip("Far clip plane in meters (values farther are clamped to maxDepth)")]
        public float farClip = 20.0f;

        [Tooltip("Value to use for invalid/sky depth")]
        public float invalidDepthValue = 0.0f;

        [Header("Compute Shader")]
        [Tooltip("Compute shader for depth linearization")]
        public ComputeShader linearizeDepthShader;

        [Header("Options")]
        [Tooltip("Enable CPU readback for debugging (slower)")]
        public bool enableCpuReadback = false;

        #endregion

        #region Private Fields

        private RenderTexture _depthTexture;
        private Texture2D _cpuTexture;
        private int _kernelId = -1;
        private bool _isInitialized = false;

        // Shader property IDs (cached for performance)
        private static readonly int _DepthInputId = Shader.PropertyToID("_DepthInput");
        private static readonly int _LinearDepthOutputId = Shader.PropertyToID("_LinearDepthOutput");
        private static readonly int _ZBufferParamsId = Shader.PropertyToID("_ZBufferParams");
        private static readonly int _DepthRangeId = Shader.PropertyToID("_DepthRange");
        private static readonly int _InvalidDepthId = Shader.PropertyToID("_InvalidDepth");
        private static readonly int _TextureSizeId = Shader.PropertyToID("_TextureSize");

        #endregion

        #region Properties

        /// <summary>
        /// The linearized depth RenderTexture for CUDA registration.
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
        /// Native texture pointer for CUDA registration.
        /// </summary>
        public IntPtr NativeTexturePtr => HasTexture ? _depthTexture.GetNativeTexturePtr() : IntPtr.Zero;

        #endregion

        #region CustomPass Overrides

        protected override void Setup(ScriptableRenderContext renderContext, CommandBuffer cmd)
        {
            name = "Linear Depth Capture";

            if (linearizeDepthShader == null)
            {
                Debug.LogError("[LinearDepthCapturePass] Compute shader not assigned");
                return;
            }

            // Find kernel
            _kernelId = linearizeDepthShader.FindKernel("LinearizeDepth");
            if (_kernelId < 0)
            {
                Debug.LogError("[LinearDepthCapturePass] Kernel 'LinearizeDepth' not found in compute shader");
                return;
            }

            // Create output texture
            CreateDepthTexture();

            _isInitialized = true;
            Debug.Log($"[LinearDepthCapturePass] Setup complete: {width}x{height}, " +
                      $"NativePtr=0x{NativeTexturePtr.ToInt64():X}");
        }

        protected override void Execute(CustomPassContext ctx)
        {
            if (!_isInitialized || _kernelId < 0)
            {
                return;
            }

            // Ensure texture exists and matches size
            if (!HasTexture || _depthTexture.width != width || _depthTexture.height != height)
            {
                CreateDepthTexture();
            }

            // Get ZBufferParams from the camera
            var camera = ctx.hdCamera.camera;
            Vector4 zBufferParams = CalculateZBufferParams(camera);

            // Set compute shader parameters
            // Note: HDRP's cameraDepthBuffer is a Texture2DArray, we sample slice 0 (main view)
            int sliceIndex = 0;
            ctx.cmd.SetComputeTextureParam(linearizeDepthShader, _kernelId, _DepthInputId, ctx.cameraDepthBuffer);
            ctx.cmd.SetComputeTextureParam(linearizeDepthShader, _kernelId, _LinearDepthOutputId, _depthTexture);
            ctx.cmd.SetComputeVectorParam(linearizeDepthShader, _ZBufferParamsId, zBufferParams);
            ctx.cmd.SetComputeVectorParam(linearizeDepthShader, _DepthRangeId, new Vector4(nearClip, farClip, 0, 0));
            ctx.cmd.SetComputeFloatParam(linearizeDepthShader, _InvalidDepthId, invalidDepthValue);
            ctx.cmd.SetComputeVectorParam(linearizeDepthShader, _TextureSizeId, new Vector4(width, height, sliceIndex, 0));

            // Dispatch compute shader
            // Thread group size is 8x8, so we need (width/8) x (height/8) groups
            int groupsX = Mathf.CeilToInt(width / 8.0f);
            int groupsY = Mathf.CeilToInt(height / 8.0f);

            ctx.cmd.DispatchCompute(linearizeDepthShader, _kernelId, groupsX, groupsY, 1);
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
        /// </summary>
        public Texture2D ReadToCpu()
        {
            if (!enableCpuReadback)
            {
                Debug.LogWarning("[LinearDepthCapturePass] CPU readback is disabled");
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

            // Create CUDA-compatible depth texture
            // - RFloat format (single-channel 32-bit float)
            // - enableRandomWrite for compute shader output
            // - No MSAA
            _depthTexture = new RenderTexture(width, height, 0, RenderTextureFormat.RFloat)
            {
                enableRandomWrite = true,
                useMipMap = false,
                autoGenerateMips = false,
                filterMode = FilterMode.Point,
                wrapMode = TextureWrapMode.Clamp,
                name = $"LinearDepthCapture_{width}x{height}"
            };

            if (!_depthTexture.Create())
            {
                Debug.LogError("[LinearDepthCapturePass] Failed to create depth RenderTexture");
            }
        }

        private Vector4 CalculateZBufferParams(UnityEngine.Camera camera)
        {
            // Unity's _ZBufferParams calculation
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
