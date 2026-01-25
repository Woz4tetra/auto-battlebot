// Auto-Battlebot Simulation System
// Test utilities for shared memory testing using reflection

using System;
using System.IO.MemoryMappedFiles;
using System.Reflection;
using AutoBattlebot.Communication;

namespace AutoBattlebot.Tests.Editor
{
    /// <summary>
    /// Test utilities for interacting with SharedMemoryReader internals.
    /// Uses reflection to access private fields for testing purposes only.
    /// </summary>
    public static class SharedMemoryTestUtils
    {
        private static readonly FieldInfo AccessorField;
        
        static SharedMemoryTestUtils()
        {
            // Cache the reflection lookup for performance
            AccessorField = typeof(SharedMemoryReader).GetField(
                "_accessor", 
                BindingFlags.NonPublic | BindingFlags.Instance);
            
            if (AccessorField == null)
            {
                throw new InvalidOperationException(
                    "Could not find _accessor field on SharedMemoryReader. " +
                    "Has the internal implementation changed?");
            }
        }
        
        /// <summary>
        /// Writes a velocity command to the shared memory region used by a SharedMemoryReader.
        /// This bypasses normal access patterns for testing purposes.
        /// </summary>
        /// <param name="reader">The reader whose backing memory to write to.</param>
        /// <param name="command">The command to write.</param>
        /// <returns>True if write was successful.</returns>
        public static bool WriteCommand(SharedMemoryReader reader, VelocityCommand command)
        {
            if (reader == null)
            {
                throw new ArgumentNullException(nameof(reader));
            }
            
            if (!reader.IsInitialized)
            {
                throw new InvalidOperationException("SharedMemoryReader is not initialized");
            }
            
            var accessor = (MemoryMappedViewAccessor)AccessorField.GetValue(reader);
            if (accessor == null)
            {
                throw new InvalidOperationException("Accessor is null - reader may be disposed");
            }
            
            try
            {
                accessor.Write(0, command.CommandId);
                accessor.Write(8, command.LinearX);
                accessor.Write(16, command.LinearY);
                accessor.Write(24, command.AngularZ);
                return true;
            }
            catch (Exception)
            {
                return false;
            }
        }
        
        /// <summary>
        /// Creates a mock velocity command for testing.
        /// </summary>
        /// <param name="commandId">Command ID (defaults to 1).</param>
        /// <param name="linearX">Linear X velocity (defaults to 1.0).</param>
        /// <param name="linearY">Linear Y velocity (defaults to 0.0).</param>
        /// <param name="angularZ">Angular Z velocity (defaults to 0.5).</param>
        /// <returns>A configured VelocityCommand.</returns>
        public static VelocityCommand CreateMockCommand(
            ulong commandId = 1,
            double linearX = 1.0,
            double linearY = 0.0,
            double angularZ = 0.5)
        {
            return new VelocityCommand
            {
                CommandId = commandId,
                LinearX = linearX,
                LinearY = linearY,
                AngularZ = angularZ
            };
        }
        
        /// <summary>
        /// Creates an invalid command with NaN values for testing validation.
        /// </summary>
        public static VelocityCommand CreateNaNCommand(ulong commandId = 1)
        {
            return new VelocityCommand
            {
                CommandId = commandId,
                LinearX = double.NaN,
                LinearY = 0.0,
                AngularZ = 0.0
            };
        }
        
        /// <summary>
        /// Creates an invalid command with excessive velocity for testing validation.
        /// </summary>
        public static VelocityCommand CreateExcessiveVelocityCommand(
            ulong commandId = 1, 
            double velocity = 100.0)
        {
            return new VelocityCommand
            {
                CommandId = commandId,
                LinearX = velocity,
                LinearY = 0.0,
                AngularZ = 0.0
            };
        }
    }
}
