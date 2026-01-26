using UnityEngine;
using TMPro;

namespace AutoBattlebot.SimulationUI
{
    /// <summary>
    /// Displays the current time since startup in milliseconds on a TextMeshPro text object.
    /// Useful for measuring end-to-end latency in the simulation pipeline.
    /// </summary>
    public class LatencyTimer : MonoBehaviour
    {
        [SerializeField]
        [Tooltip("The TextMeshPro text component to update. If not assigned, will search on this GameObject.")]
        private TMP_Text _text;

        [SerializeField]
        [Tooltip("Update frequency in Hz (updates per second). Higher = more precise but more CPU usage.")]
        private float _updateFrequency = 60f;

        [SerializeField]
        [Tooltip("Format string for the time display. {0} = milliseconds since startup.")]
        private string _format = "{0:F0} ms";

        [SerializeField]
        [Tooltip("Text color for the timer display.")]
        private Color _textColor = Color.white;

        [SerializeField]
        [Tooltip("Optional: Add a background for better visibility.")]
        private bool _showBackground = true;

        private float _updateInterval;
        private float _lastUpdateTime;

        private void Awake()
        {
            if (_text == null)
            {
                _text = GetComponent<TMP_Text>();
            }

            if (_text == null)
            {
                Debug.LogError("[LatencyTimer] No TMP_Text component found!");
                enabled = false;
                return;
            }

            _text.color = _textColor;
            _updateInterval = 1f / _updateFrequency;
        }

        private void Update()
        {
            float currentTime = Time.realtimeSinceStartup;

            if (currentTime - _lastUpdateTime >= _updateInterval)
            {
                float milliseconds = currentTime * 1000f;
                _text.text = string.Format(_format, milliseconds);
                _lastUpdateTime = currentTime;
            }
        }

        private void OnValidate()
        {
            if (_updateFrequency < 1f)
            {
                _updateFrequency = 1f;
            }
            else if (_updateFrequency > 1000f)
            {
                _updateFrequency = 1000f;
            }
        }
    }
}
