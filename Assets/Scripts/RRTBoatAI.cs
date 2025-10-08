using UnityEngine;
using System.Collections.Generic;
using RRT;
using Crest;

[RequireComponent(typeof(BoatProbesAI))]
public class RRTBoatAI : MonoBehaviour
{
    [Header("AI 컨트롤러 참조")]
    public RRT.RRT _rrtController;
    private BoatProbesAI _boatProbes;

    [Header("동적 장애물 감지")]
    [Tooltip("동적 장애물을 감지할 반경")]
    public float detectionRadius = 30f;
    [Tooltip("동적 장애물이 있는 레이어")]
    public LayerMask dynamicObstacleLayerMask;
    [Tooltip("재탐색 요청 사이의 최소 시간 간격")]
    public float replanCooldown = 2f;
    [Tooltip("장애물의 미래 위치를 예측할 시간")]
    public float predictionTime = 1.5f;
    [Tooltip("미래 위치 예측에 사용할 임시 장애물 오브젝트")]
    public Transform predictionMarker;

    [Header("경로 추적 설정")]
    [Tooltip("웨이포인트 도달 판정 거리")]
    public float waypointReachDistance = 5f;
    [Tooltip("회전 감도 (각도를 입력값으로 변환)")]
    public float turnSensitivity = 45f;

    [Header("디버그")]
    public bool showDebugInfo = true;
    public Color pathColor = Color.green;

    private List<Vector3> _waypoints;
    private int _currentWaypointIndex = 0;
    private bool _hasPath = false;
    private float _lastReplanTime = -100f;

    void Awake()
    {
        _boatProbes = GetComponent<BoatProbesAI>();
        
        // RRT 컨트롤러가 없으면 찾기
        if (_rrtController == null)
        {
            _rrtController = GetComponent<RRT.RRT>();
        }
    }

    void Start()
    {
        // RRT 이벤트에 경로 수신 함수 연결
        if (_rrtController != null)
        {
            _rrtController.OnPathFound.AddListener(OnPathFound);
            Debug.Log("RRTBoatAI: RRT OnPathFound 이벤트 연결 완료");
        }
        else
        {
            Debug.LogError("RRTBoatAI: RRT 컨트롤러를 찾을 수 없습니다!");
        }
    }

    // RRT.cs가 경로를 찾았을 때 호출될 함수
    public void OnPathFound()
    {
        Debug.Log("RRTBoatAI: 경로 발견 이벤트 수신!");
        ExtractPathFromTree();
    }

    void FixedUpdate()
    {
        Sense(); // 1. 주변 감지
        Act();   // 2. 행동 결정 및 실행
    }

    // 주변의 동적 장애물을 감지하는 함수
    void Sense()
    {
        // 재탐색 쿨다운 중이거나 경로가 없으면 감지 안 함
        if (Time.time < _lastReplanTime + replanCooldown || !_hasPath) return;

        Collider[] hits = Physics.OverlapSphere(transform.position, detectionRadius, dynamicObstacleLayerMask);

        foreach (var hit in hits)
        {
            if (hit.transform == this.transform) continue;

            // 동적 장애물 감지 시, 미래 위치를 예측하고 재탐색 요청
            Rigidbody obsRb = hit.GetComponent<Rigidbody>();
            if (obsRb != null)
            {
                Vector3 predictedPosition = hit.transform.position + obsRb.velocity * predictionTime;
                
                // 예측 마커를 미래 위치로 이동시키고 재탐색 요청
                if (predictionMarker != null)
                {
                    predictionMarker.position = predictedPosition;
                    _rrtController.RequestReplanning();
                    _lastReplanTime = Time.time;
                    _hasPath = false; // 현재 경로 무효화
                    Debug.LogWarning($"동적 장애물({hit.name}) 감지! {predictionTime}초 뒤의 위치를 예측하여 재탐색합니다.");
                    return; // 한 번에 하나의 장애물에만 반응
                }
            }
        }
    }

    // 경로를 따라 보트를 움직이는 함수
    void Act()
    {
        if (!_hasPath || _waypoints == null || _waypoints.Count == 0)
        {
            _boatProbes.AI_ForwardInput = 0;
            _boatProbes.AI_TurnInput = 0;
            
            if (showDebugInfo && _rrtController != null && _rrtController._tree != null)
            {
                Debug.Log($"대기 중 - HasPath: {_hasPath}, Tree HasFoundPath: {_rrtController._tree.HasFoundPath}");
            }
            return;
        }

        if (_currentWaypointIndex >= _waypoints.Count)
        {
            Debug.Log("목표 지점 도착!");
            _boatProbes.AI_ForwardInput = 0;
            _boatProbes.AI_TurnInput = 0;
            _hasPath = false;
            return;
        }

        Vector3 targetWaypoint = _waypoints[_currentWaypointIndex];
        Vector3 horizontalTarget = new Vector3(targetWaypoint.x, transform.position.y, targetWaypoint.z);

        // 1. 전진 입력
        _boatProbes.AI_ForwardInput = 1f;

        // 2. 회전 입력
        Vector3 directionToWaypoint = (horizontalTarget - transform.position).normalized;
        float angle = Vector3.SignedAngle(transform.forward, directionToWaypoint, Vector3.up);
        _boatProbes.AI_TurnInput = Mathf.Clamp(angle / turnSensitivity, -1f, 1f);

        // 디버그 정보
        if (showDebugInfo)
        {
            Debug.DrawLine(transform.position, targetWaypoint, Color.yellow);
            Debug.DrawRay(transform.position, transform.forward * 5f, Color.blue);
        }

        // 웨이포인트 도착 시 다음으로 이동
        float distanceToWaypoint = Vector3.Distance(transform.position, horizontalTarget);
        if (distanceToWaypoint < waypointReachDistance)
        {
            Debug.Log($"웨이포인트 {_currentWaypointIndex} 도착! 다음 웨이포인트로 이동");
            _currentWaypointIndex++;
        }
    }
    
    // RRT 트리에서 경로를 추출하는 함수
    void ExtractPathFromTree()
    {
        if (_rrtController == null || _rrtController._tree == null)
        {
            Debug.LogError("RRT 컨트롤러 또는 트리가 없습니다!");
            return;
        }

        if (!_rrtController._tree.HasFoundPath)
        {
            Debug.LogWarning("아직 경로를 찾지 못했습니다.");
            return;
        }

        Debug.Log("새로운 경로 추출 시작!");
        _waypoints = new List<Vector3>();
        Node currentNode = _rrtController._tree.TargetNode;
        
        if (currentNode == null)
        {
            Debug.LogError("TargetNode가 null입니다!");
            return;
        }

        // 경로 추출
        while (currentNode != null)
        {
            _waypoints.Add(currentNode.Position);
            currentNode = currentNode.Parent;
        }
        
        _waypoints.Reverse();
        
        Debug.Log($"경로 추출 완료! 총 {_waypoints.Count}개의 웨이포인트");
        
        _currentWaypointIndex = 0;
        _hasPath = true;

        // 재탐색 후 예측 마커를 멀리 치워 다음 스캔에 영향 없도록 함
        if (predictionMarker != null)
        {
            predictionMarker.position = new Vector3(0, -1000, 0);
        }
    }

    // 경로 시각화
    void OnDrawGizmos()
    {
        if (!showDebugInfo || _waypoints == null || _waypoints.Count == 0) return;

        Gizmos.color = pathColor;
        for (int i = 0; i < _waypoints.Count - 1; i++)
        {
            Gizmos.DrawLine(_waypoints[i], _waypoints[i + 1]);
            Gizmos.DrawSphere(_waypoints[i], 0.3f);
        }
        
        if (_waypoints.Count > 0)
        {
            Gizmos.DrawSphere(_waypoints[_waypoints.Count - 1], 0.5f);
        }

        // 현재 목표 웨이포인트 강조
        if (_hasPath && _currentWaypointIndex < _waypoints.Count)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(_waypoints[_currentWaypointIndex], 0.7f);
        }
    }

    void OnDestroy()
    {
        // 이벤트 리스너 제거
        if (_rrtController != null)
        {
            _rrtController.OnPathFound.RemoveListener(OnPathFound);
        }
    }
}