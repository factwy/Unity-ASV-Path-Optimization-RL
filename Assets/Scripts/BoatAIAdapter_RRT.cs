using UnityEngine;
using Pathfinding;
using Crest;
using RRT;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

/// <summary>
/// RRT 경로를 AIPath의 desiredVelocity 스타일로 추적하는 어댑터
/// A* AIPath의 이동 방식을 모방하되, 경로는 RRT*로 찾습니다.
/// </summary>
[RequireComponent(typeof(BoatProbes_AStar))]
public class BoatAIAdapter_RRT : MonoBehaviour
{
    [Header("RRT 경로 찾기")]
    public RRT.RRT rrtController;
    public Transform target;

    [Header("AIPath 스타일 설정")]
    [Tooltip("최대 속도 (m/s)")]
    public float maxSpeed = 5f;
    
    [Tooltip("다음 웨이포인트 거리")]
    public float nextWaypointDistance = 8f;
    
    [Tooltip("목표 도착 거리")]
    public float endReachedDistance = 12f;
    
    [Tooltip("회전 속도")]
    public float rotationSpeed = 120f;

    [Header("디버그")]
    public bool showDebugInfo = true;
    public Color pathColor = Color.green;

    // --- 타이머 및 충돌 카운터 ---
    private float _startTime;
    private bool _isTimerRunning;
    private int _collisionCount = 0;

    // --- RRT 경로 관리 ---
    private List<Vector3> _rrtPath;
    private int _currentWaypoint = 0;
    private bool _hasValidPath = false;
    private bool _reachedDestination = false;

    // --- 컴포넌트 참조 ---
    private BoatProbes_AStar _boatProbes;
    private Rigidbody _rb;

    // --- AIPath 스타일 속성 (호환성) ---
    public Vector3 desiredVelocity { get; private set; }
    public bool reachedDestination => _reachedDestination;
    public bool hasPath => _hasValidPath;

    void Awake()
    {
        _boatProbes = GetComponent<BoatProbes_AStar>();
        _rb = GetComponent<Rigidbody>();

        if (rrtController == null)
        {
            rrtController = GetComponent<RRT.RRT>();
        }
    }

    void Start()
    {
        // RRT 이벤트 연결
        if (rrtController != null)
        {
            rrtController.OnPathFound.AddListener(OnRRTPathFound);
            Debug.Log("BoatAIAdapter_RRT: RRT 이벤트 연결 완료");
        }
        else
        {
            Debug.LogError("RRT 컨트롤러를 찾을 수 없습니다!");
        }

        // 타이머 시작
        if (target != null)
        {
            StartTimer();
        }
    }

    void StartTimer()
    {
        _startTime = Time.time;
        _isTimerRunning = true;
        Debug.Log("이동 시작! RRT* 경로 탐색 중...");
    }

    /// <summary>
    /// RRT*가 경로를 찾았을 때 호출
    /// </summary>
    public void OnRRTPathFound()
    {
        Debug.Log("<color=yellow>RRT* 경로 발견!</color>");

        _rrtPath = ExtractPathFromRRT();

        if (_rrtPath == null || _rrtPath.Count == 0)
        {
            Debug.LogError("RRT 경로 추출 실패!");
            return;
        }

        _currentWaypoint = 0;
        _hasValidPath = true;
        _reachedDestination = false;

        Debug.Log($"<color=cyan>경로 추출 완료! {_rrtPath.Count}개 웨이포인트</color>");
    }

    /// <summary>
    /// RRT 트리에서 경로 추출
    /// </summary>
    List<Vector3> ExtractPathFromRRT()
    {
        if (rrtController == null || rrtController._tree == null)
        {
            Debug.LogError("RRT 컨트롤러 또는 트리가 없습니다!");
            return null;
        }

        if (!rrtController._tree.HasFoundPath)
        {
            Debug.LogWarning("아직 경로를 찾지 못했습니다.");
            return null;
        }

        List<Vector3> path = new List<Vector3>();
        Node currentNode = rrtController._tree.TargetNode;

        if (currentNode == null)
        {
            Debug.LogError("TargetNode가 null입니다!");
            return null;
        }

        // 경로 추출 (역순)
        while (currentNode != null)
        {
            path.Add(currentNode.Position);
            currentNode = currentNode.Parent;
        }

        path.Reverse();
        return path;
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Obstacle"))
        {
            _collisionCount++;
            Debug.Log($"장애물과 충돌! (현재 {_collisionCount}회)");
        }
    }

    void Update()
    {
        // --- AIPath 스타일의 desiredVelocity 계산 ---
        CalculateDesiredVelocity();

        // --- BoatProbes 제어 (기존 BoatAIAdapter 방식) ---
        if (desiredVelocity.magnitude > 0.1f)
        {
            // 1. 전진 입력
            float forwardInput = desiredVelocity.magnitude / maxSpeed;
            _boatProbes.AI_ForwardInput = Mathf.Clamp01(forwardInput);

            // 2. 회전 입력
            float angle = Vector3.SignedAngle(transform.forward, desiredVelocity, Vector3.up);
            _boatProbes.AI_TurnInput = Mathf.Clamp(angle / 45f, -1f, 1f);

            if (showDebugInfo && Time.frameCount % 60 == 0)
            {
                Debug.Log($"Desired Velocity: {desiredVelocity.magnitude:F2}, Forward: {forwardInput:F2}, Turn: {_boatProbes.AI_TurnInput:F2}");
            }
        }
        else
        {
            _boatProbes.AI_ForwardInput = 0;
            _boatProbes.AI_TurnInput = 0;
        }

        // --- 목표 도착 확인 ---
        if (_isTimerRunning && _reachedDestination)
        {
            _isTimerRunning = false;
            float elapsedTime = Time.time - _startTime;

            Debug.Log($"<color=cyan>=== 목표 지점 도착! ===\n" +
                      $"총 소요 시간: {elapsedTime:F2}초\n" +
                      $"장애물 충돌 횟수: {_collisionCount}회\n" +
                      $"경로 길이: {CalculatePathLength():F2}m</color>");

            #if UNITY_EDITOR
            EditorApplication.isPlaying = false;
            #endif
        }
    }

    /// <summary>
    /// AIPath 스타일의 desiredVelocity 계산
    /// </summary>
    void CalculateDesiredVelocity()
    {
        // 경로가 없으면 정지
        if (!_hasValidPath || _rrtPath == null || _rrtPath.Count == 0)
        {
            desiredVelocity = Vector3.zero;
            return;
        }

        // 모든 웨이포인트를 통과했으면 정지
        if (_currentWaypoint >= _rrtPath.Count)
        {
            desiredVelocity = Vector3.zero;
            _reachedDestination = true;
            return;
        }

        // 현재 위치 (Y축 무시)
        Vector3 currentPos = new Vector3(transform.position.x, 0, transform.position.z);
        Vector3 waypointPos = new Vector3(_rrtPath[_currentWaypoint].x, 0, _rrtPath[_currentWaypoint].z);

        // 웨이포인트까지의 거리
        float distanceToWaypoint = Vector3.Distance(currentPos, waypointPos);

        // 다음 웨이포인트로 이동
        if (distanceToWaypoint < nextWaypointDistance)
        {
            _currentWaypoint++;
            
            if (_currentWaypoint >= _rrtPath.Count)
            {
                desiredVelocity = Vector3.zero;
                _reachedDestination = true;
                return;
            }

            Debug.Log($"웨이포인트 {_currentWaypoint - 1} 통과!");
            waypointPos = new Vector3(_rrtPath[_currentWaypoint].x, 0, _rrtPath[_currentWaypoint].z);
        }

        // 목표까지의 거리
        Vector3 targetPos = new Vector3(target.position.x, 0, target.position.z);
        float distanceToTarget = Vector3.Distance(currentPos, targetPos);

        // 목표 도착 확인
        if (distanceToTarget < endReachedDistance)
        {
            desiredVelocity = Vector3.zero;
            _reachedDestination = true;
            return;
        }

        // desiredVelocity 계산 (AIPath 스타일)
        Vector3 direction = (waypointPos - currentPos).normalized;
        float speed = maxSpeed;

        // 목표 근처에서 감속
        if (distanceToTarget < endReachedDistance * 2f)
        {
            speed *= Mathf.Clamp01(distanceToTarget / (endReachedDistance * 2f));
        }

        desiredVelocity = direction * speed;
    }

    /// <summary>
    /// 경로의 총 길이 계산
    /// </summary>
    float CalculatePathLength()
    {
        if (_rrtPath == null || _rrtPath.Count < 2) return 0f;

        float length = 0f;
        for (int i = 0; i < _rrtPath.Count - 1; i++)
        {
            length += Vector3.Distance(_rrtPath[i], _rrtPath[i + 1]);
        }
        return length;
    }

    void OnDrawGizmos()
    {
        if (!showDebugInfo || _rrtPath == null || _rrtPath.Count == 0)
            return;

        // 경로 그리기
        Gizmos.color = pathColor;
        for (int i = 0; i < _rrtPath.Count - 1; i++)
        {
            Gizmos.DrawLine(_rrtPath[i], _rrtPath[i + 1]);
            Gizmos.DrawWireSphere(_rrtPath[i], 0.3f);
        }

        // 현재 목표 웨이포인트
        if (_hasValidPath && _currentWaypoint < _rrtPath.Count)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(_rrtPath[_currentWaypoint], 0.6f);
            Gizmos.DrawLine(transform.position, _rrtPath[_currentWaypoint]);
        }

        // desiredVelocity 방향 표시
        if (desiredVelocity.magnitude > 0.1f)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(transform.position, desiredVelocity.normalized * 3f);
        }
    }

    void OnDestroy()
    {
        if (rrtController != null)
        {
            rrtController.OnPathFound.RemoveListener(OnRRTPathFound);
        }
    }
}