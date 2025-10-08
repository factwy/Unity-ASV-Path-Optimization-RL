// Crest Ocean System

// This file is subject to the MIT License as seen in the root of this folder structure (LICENSE)

// Shout out to @holdingjason who posted a first version of this script here: https://github.com/huwb/crest-oceanrender/pull/100

#if CREST_UNITY_INPUT && ENABLE_INPUT_SYSTEM
#define INPUT_SYSTEM_ENABLED
#endif

using Crest.Internal;
using System;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Serialization;

namespace Crest
{
    /// <summary>
    /// Boat physics by sampling at multiple probe points.
    /// </summary>
    [AddComponentMenu(Internal.Constants.MENU_PREFIX_SCRIPTS + "Boat Probes AStar")]
    public class BoatProbes_RL : FloatingObjectBase // <-- 1. 클래스 이름 변경
    {
        // ... (기존 변수들은 그대로 유지) ...
        [Header("Forces")]
        [Tooltip("Override RB center of mass, in local space."), SerializeField]
        Vector3 _centerOfMass = Vector3.zero;
        [SerializeField, FormerlySerializedAs("ForcePoints")]
        FloaterForcePoints[] _forcePoints = new FloaterForcePoints[] { };
        [Tooltip("Vertical offset for where engine force should be applied.")]
        public float _forceHeightOffset = 0f;
        public float _forceMultiplier = 10f;
        [Tooltip("Width dimension of boat. The larger this value, the more filtered/smooth the wave response will be.")]
        public float _minSpatialLength = 12f;
        [Range(0, 1)]
        public float _turningHeel = 0.35f;
        [Tooltip("Clamps the buoyancy force to this value. Useful for handling fully submerged objects. Enter 'Infinity' to disable.")]
        public float _maximumBuoyancyForce = Mathf.Infinity;

        [Header("Drag")]
        public float _dragInWaterUp = 3f;
        public float _dragInWaterRight = 2f;
        public float _dragInWaterForward = 1f;

        [Header("Control")]
        [FormerlySerializedAs("EnginePower")]
        public float _enginePower = 7;
        [FormerlySerializedAs("TurnPower")]
        public float _turnPower = 0.5f;
        public bool _playerControlled = true;
        [Tooltip("Used to automatically add throttle input")]
        public float _engineBias = 0f;
        [Tooltip("Used to automatically add turning input")]
        public float _turnBias = 0f;

        // <-- 2. AI 제어를 위한 public 변수 추가
        [HideInInspector] public float AI_ForwardInput = 0f;
        [HideInInspector] public float AI_TurnInput = 0f;

        [Space(10)]
        [SerializeField]
        DebugFields _debug = new DebugFields();
        // ... (나머지 변수 및 클래스 선언은 그대로 유지) ...
#pragma warning disable 414
        [SerializeField, HideInInspector] int _version = 0;
#pragma warning restore 414
        [Serializable]
        class DebugFields { [Tooltip("Draw queries for each force point as gizmos.")] public bool _drawQueries = false; }
        private const float WATER_DENSITY = 1000;
        public override Vector3 Velocity => _rb.velocity;
        Rigidbody _rb;
        public override float ObjectWidth => _minSpatialLength;
        public override bool InWater => true;
        float _totalWeight;
        Vector3[] _queryPoints;
        Vector3[] _queryResultDisps;
        Vector3[] _queryResultVels;
        SampleFlowHelper _sampleFlowHelper = new SampleFlowHelper();

        private void Start()
        {
            _rb = GetComponent<Rigidbody>();
            _rb.centerOfMass = _centerOfMass;
            CalcTotalWeight();
            _queryPoints = new Vector3[_forcePoints.Length + 1];
            _queryResultDisps = new Vector3[_forcePoints.Length + 1];
            _queryResultVels = new Vector3[_forcePoints.Length + 1];
        }

        void CalcTotalWeight()
        {
            _totalWeight = 0f;
            foreach (var pt in _forcePoints) { _totalWeight += pt._weight; }
        }
        
        // FixedUpdate, UpdateWaterQueries, FixedUpdateBuoyancy 등 다른 메서드들은 수정 없이 그대로 둡니다.
        // ...

        void FixedUpdateEngine()
        {
            var forcePosition = _rb.worldCenterOfMass;

            // --- Forward/Backward Movement ---
            var forward = _engineBias;
            if (_playerControlled)
            {
                // Player Control
                forward +=
#if INPUT_SYSTEM_ENABLED
                !Application.isFocused ? 0 :
                ((Keyboard.current.wKey.isPressed ? 1 : 0) + (Keyboard.current.sKey.isPressed ? -1 : 0));
#else
                Input.GetAxis("Vertical");
#endif
            }
            else
            {
                // <-- 3. AI 제어 로직 추가
                // forward += AI_ForwardInput;
                forward *= AI_ForwardInput;
            }
            _rb.AddForceAtPosition(_enginePower * forward * transform.forward, forcePosition, ForceMode.Acceleration);

            // --- Turning Movement ---
            var sideways = _turnBias;
            if (_playerControlled)
            {
                // Player Control
                sideways +=
#if INPUT_SYSTEM_ENABLED
                !Application.isFocused ? 0 :
                ((Keyboard.current.aKey.isPressed ? -1f : 0f) +
                (Keyboard.current.dKey.isPressed ? 1f : 0f));
#else
                (Input.GetKey(KeyCode.A) ? -1f : 0f) +
                (Input.GetKey(KeyCode.D) ? 1f : 0f);
#endif
            }
            else
            {
                // <-- 4. AI 제어 로직 추가
                // sideways += AI_TurnInput;
                sideways *= AI_TurnInput;
            }
            var rotVec = transform.up + _turningHeel * transform.forward;
            _rb.AddTorque(_turnPower * sideways * rotVec, ForceMode.Acceleration);
        }

        // ... (FixedUpdateBuoyancy, FixedUpdateDrag, OnDrawGizmosSelected 등 나머지 메서드들은 그대로 유지) ...
        private void FixedUpdate() { if (OceanRenderer.Instance == null) { return; } var collProvider = OceanRenderer.Instance.CollisionProvider; UpdateWaterQueries(collProvider); var undispPos = transform.position - _queryResultDisps[_forcePoints.Length]; undispPos.y = OceanRenderer.Instance.SeaLevel; var waterSurfaceVel = _queryResultVels[_forcePoints.Length]; { _sampleFlowHelper.Init(transform.position, _minSpatialLength); _sampleFlowHelper.Sample(out var surfaceFlow); waterSurfaceVel += new Vector3(surfaceFlow.x, 0, surfaceFlow.y); } if (_debug._drawQueries) { Debug.DrawLine(transform.position + 5f * Vector3.up, transform.position + 5f * Vector3.up + waterSurfaceVel, new Color(1, 1, 1, 0.6f)); } FixedUpdateBuoyancy(); FixedUpdateDrag(waterSurfaceVel); FixedUpdateEngine(); }
        void UpdateWaterQueries(ICollProvider collProvider) { for (int i = 0; i < _forcePoints.Length; i++) { _queryPoints[i] = transform.TransformPoint(_forcePoints[i]._offsetPosition + new Vector3(0, _centerOfMass.y, 0)); } _queryPoints[_forcePoints.Length] = transform.position; collProvider.Query(GetHashCode(), ObjectWidth, _queryPoints, _queryResultDisps, null, _queryResultVels); if (_debug._drawQueries) { for (var i = 0; i < _forcePoints.Length; i++) { var query = _queryPoints[i]; query.y = OceanRenderer.Instance.SeaLevel + _queryResultDisps[i].y; VisualiseCollisionArea.DebugDrawCross(query, 1f, Color.magenta); } } }
        void FixedUpdateBuoyancy() { var archimedesForceMagnitude = WATER_DENSITY * Mathf.Abs(Physics.gravity.y); for (int i = 0; i < _forcePoints.Length; i++) { var waterHeight = OceanRenderer.Instance.SeaLevel + _queryResultDisps[i].y; var heightDiff = waterHeight - _queryPoints[i].y; if (heightDiff > 0) { var force = _forceMultiplier * _forcePoints[i]._weight * archimedesForceMagnitude * heightDiff * Vector3.up / _totalWeight; if (_maximumBuoyancyForce < Mathf.Infinity) { force = Vector3.ClampMagnitude(force, _maximumBuoyancyForce); } _rb.AddForceAtPosition(force, _queryPoints[i]); } } }
        void FixedUpdateDrag(Vector3 waterSurfaceVel) { var _velocityRelativeToWater = Velocity - waterSurfaceVel; var forcePosition = _rb.worldCenterOfMass + _forceHeightOffset * Vector3.up; _rb.AddForceAtPosition(_dragInWaterUp * Vector3.Dot(Vector3.up, -_velocityRelativeToWater) * Vector3.up, forcePosition, ForceMode.Acceleration); _rb.AddForceAtPosition(_dragInWaterRight * Vector3.Dot(transform.right, -_velocityRelativeToWater) * transform.right, forcePosition, ForceMode.Acceleration); _rb.AddForceAtPosition(_dragInWaterForward * Vector3.Dot(transform.forward, -_velocityRelativeToWater) * transform.forward, forcePosition, ForceMode.Acceleration); }
        private void OnDrawGizmosSelected() { Gizmos.color = Color.yellow; Gizmos.DrawCube(transform.TransformPoint(_centerOfMass), Vector3.one * 0.25f); for (int i = 0; i < _forcePoints.Length; i++) { var point = _forcePoints[i]; var transformedPoint = transform.TransformPoint(point._offsetPosition + new Vector3(0, _centerOfMass.y, 0)); Gizmos.color = Color.red; Gizmos.DrawCube(transformedPoint, Vector3.one * 0.5f); } }
    }
}