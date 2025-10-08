using UnityEngine;
using Pathfinding;
using Crest;

#if UNITY_EDITOR
using UnityEditor;
#endif

/// <summary>
/// RRT* 스타일의 자연스러운 움직임을 적용한 A* 보트 컨트롤러
/// </summary>
[RequireComponent(typeof(BoatProbes_AStar), typeof(Seeker))]
public class BoatAIController_AStar_Enhanced : MonoBehaviour
{
    [Header("Target")]
    public Transform target;

    [Header("Pathfinding")]
    [Tooltip("다음 웨이포인트까지의 거리")]
    public float nextWaypointDistance = 8f;
    
    [Tooltip("경로 업데이트 주기")]
    public float pathUpdateInterval = 0.5f;

    [Header("Movement Control")]
    [Tooltip("목표 지점 정지 거리")]
    public float stoppingDistance = 12f;
    
    [Tooltip("감속 시작 각도")]
    public float slowDownAngle = 45f;
    
    [Tooltip("최대 전진 입력")]
    public float maxSpeedInput = 1f;

    [Header("Movement Enhancement (RRT 스타일)")]
    [Tooltip("엔진 파워 배율")]
    [UnityEngine.Range(1f, 10f)]
    public float enginePowerMultiplier = 3f;
    
    [Tooltip("AI 입력값 부스트")]
    [UnityEngine.Range(1f, 3f)]
    public float inputBoost = 1.5f;
    
    [Tooltip("물 저항 감소 배율")]
    [UnityEngine.Range(0.1f, 1f)]
    public float dragReduction = 0.5f;
    
    [Tooltip("회전 중 속도 유지")]
    [UnityEngine.Range(0f, 1f)]
    public float speedDuringTurn = 0.7f;
    
    [Tooltip("최소 전진 입력 (회전 중)")]
    [UnityEngine.Range(0f, 0.5f)]
    public float minimumForwardInput = 0.2f;

    [Header("Path Visualization")]
    public PathToImage_Lines pathToImageGenerator;

    [Header("Debug")]
    public bool showDebugInfo = true;

    // 타이머 및 충돌 카운터
    private float _startTime;
    private bool _isTimerRunning;
    private int _collisionCount = 0;

    // A* 경로 관리
    private Path _path;
    private int _currentWaypoint = 0;

    private Seeker _seeker;
    private BoatProbes_AStar _boatProbes;
    private Rigidbody _rb;

    // 원본 값 저장 (Enhancement용)
    private float _originalEnginePower;
    private float _originalDragUp;
    private float _originalDragRight;
    private float _originalDragForward;

    void Awake()
    {
        _seeker = GetComponent<Seeker>();
        _boatProbes = GetComponent<BoatProbes_AStar>();
        _rb = GetComponent<Rigidbody>();
    }

    void Start()
    {
        // 원본 값 저장
        _originalEnginePower = _boatProbes._enginePower;
        _originalDragUp = _boatProbes._dragInWaterUp;
        _originalDragRight = _boatProbes._dragInWaterRight;
        _originalDragForward = _boatProbes._dragInWaterForward;

        // RRT 스타일 Enhancement 적용
        ApplyMovementEnhancement();

        // 경로 업데이트 시작
        InvokeRepeating(nameof(UpdatePath), 0f, pathUpdateInterval);

        // 타이머 시작
        if (target != null)
        {
            _startTime = Time.time;
            _isTimerRunning = true;
            Debug.Log("이동 시작! A* 경로 탐색 중...");
        }
    }

    /// <summary>
    /// RRT 스타일의 움직임 강화 적용
    /// </summary>
    void ApplyMovementEnhancement()
    {
        // 엔진 파워 증가
        _boatProbes._enginePower = _originalEnginePower * enginePowerMultiplier;

        // 저항 감소 (더 부드러운 움직임)
        _boatProbes._dragInWaterUp = _originalDragUp * dragReduction;
        _boatProbes._dragInWaterRight = _originalDragRight * dragReduction;
        _boatProbes._dragInWaterForward = _originalDragForward * dragReduction;

        if (showDebugInfo)
        {
            Debug.Log("=== A* 움직임 강화 적용 (RRT 스타일) ===");
            Debug.Log($"엔진 파워: {_originalEnginePower:F1} → {_boatProbes._enginePower:F1}");
            Debug.Log($"전방 저항: {_originalDragForward:F2} → {_boatProbes._dragInWaterForward:F2}");
        }
    }

    void UpdatePath()
    {
        if (target != null && _seeker.IsDone())
        {
            _seeker.StartPath(_rb.position, target.position, OnPathComplete);
        }
    }

    void OnPathComplete(Path p)
    {
        if (!p.error)
        {
            _path = p;
            _currentWaypoint = 0;
            
            if (showDebugInfo)
            {
                Debug.Log($"<color=cyan>A* 경로 발견! 웨이포인트 수: {_path.vectorPath.Count}</color>");
            }
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Obstacle"))
        {
            _collisionCount++;
            Debug.Log($"장애물과 충돌! (현재 {_collisionCount}회)");
        }
    }

    void FixedUpdate()
    {
        // 경로가 없으면 정지
        if (_path == null || _currentWaypoint >= _path.vectorPath.Count)
        {
            _boatProbes.AI_ForwardInput = 0;
            _boatProbes.AI_TurnInput = 0;
            
            if (showDebugInfo && _path == null && Time.frameCount % 120 == 0)
            {
                Debug.Log("A* 경로 대기 중...");
            }
            return;
        }

        // --- Y축을 무시한 수평 거리 계산 (RRT 방식) ---
        Vector3 currentPosition2D = new Vector3(_rb.position.x, 0, _rb.position.z);
        Vector3 waypointPosition2D = new Vector3(
            _path.vectorPath[_currentWaypoint].x, 
            0, 
            _path.vectorPath[_currentWaypoint].z
        );

        // 웨이포인트 도달 체크
        float distanceToWaypoint = Vector3.Distance(currentPosition2D, waypointPosition2D);
        if (distanceToWaypoint < nextWaypointDistance)
        {
            _currentWaypoint++;
            
            if (showDebugInfo)
            {
                Debug.Log($"웨이포인트 {_currentWaypoint - 1} 통과! (남은: {_path.vectorPath.Count - _currentWaypoint})");
            }
            
            if (_currentWaypoint >= _path.vectorPath.Count) return;
            
            // 다음 웨이포인트의 2D 위치 재계산
            waypointPosition2D = new Vector3(
                _path.vectorPath[_currentWaypoint].x, 
                0, 
                _path.vectorPath[_currentWaypoint].z
            );
        }

        // --- RRT 스타일 제어 로직 ---
        Vector3 waypointDirection = (waypointPosition2D - currentPosition2D).normalized;
        Vector3 targetPosition2D = new Vector3(target.position.x, 0, target.position.z);
        float distanceToTarget = Vector3.Distance(currentPosition2D, targetPosition2D);

        // --- 목표 지점 도착 확인 ---
        if (distanceToTarget < stoppingDistance && _isTimerRunning)
        {
            _isTimerRunning = false;
            float elapsedTime = Time.time - _startTime;

            // 경로 이미지 생성
            if (pathToImageGenerator != null && _path != null)
            {
                pathToImageGenerator.GenerateAndSaveImage(_path);
            }

            Debug.Log($"<color=cyan>=== 목표 지점 도착! ===\n" +
                      $"총 소요 시간: {elapsedTime:F2}초\n" +
                      $"장애물 충돌 횟수: {_collisionCount}회\n" +
                      $"경로 길이: {CalculatePathLength():F2}m\n" +
                      $"웨이포인트 수: {_path.vectorPath.Count}개</color>");

            #if UNITY_EDITOR
            EditorApplication.isPlaying = false;
            #endif
        }

        // 1. 회전 제어 (RRT 방식)
        float angleToWaypoint = Vector3.SignedAngle(transform.forward, waypointDirection, Vector3.up);
        _boatProbes.AI_TurnInput = Mathf.Clamp(angleToWaypoint / 45f, -1f, 1f);

        // 2. 전진 제어 (RRT 스타일 Enhancement 적용)
        float forwardInput = maxSpeedInput;
        
        // 목표 근처에서 감속
        if (distanceToTarget < stoppingDistance)
        {
            forwardInput = Mathf.Clamp01(distanceToTarget / stoppingDistance);
        }
        // 급회전 시 감속
        else if (Mathf.Abs(angleToWaypoint) > slowDownAngle)
        {
            forwardInput *= Mathf.Clamp01(1f - (Mathf.Abs(angleToWaypoint) - slowDownAngle) / 90f);
        }

        // 입력 부스트 적용
        forwardInput = ApplyInputBoost(forwardInput, angleToWaypoint);
        _boatProbes.AI_ForwardInput = forwardInput;

        // 3. 저속 시 추가 힘 적용 (RRT 방식)
        ApplyAdditionalForce(forwardInput);

        // 디버그 정보
        if (showDebugInfo && Time.frameCount % 60 == 0)
        {
            Debug.Log($"웨이포인트: {_currentWaypoint}/{_path.vectorPath.Count}, " +
                      $"거리: {distanceToWaypoint:F1}m, " +
                      $"각도: {angleToWaypoint:F1}°, " +
                      $"전진: {forwardInput:F2}, " +
                      $"속도: {_rb.velocity.magnitude:F2} m/s");
        }
    }

    /// <summary>
    /// RRT 스타일 입력 부스트 적용
    /// </summary>
    float ApplyInputBoost(float forwardInput, float angle)
    {
        if (forwardInput > 0.01f)
        {
            float boostedInput = forwardInput * inputBoost;
            boostedInput = Mathf.Clamp01(boostedInput);
            
            // 회전 중에도 최소한의 속도 유지
            if (Mathf.Abs(angle) > 30f)
            {
                boostedInput = Mathf.Max(boostedInput, speedDuringTurn);
            }
            
            return boostedInput;
        }
        else if (_boatProbes.AI_TurnInput != 0)
        {
            // 회전만 하고 있으면 최소 전진 적용
            return minimumForwardInput;
        }
        
        return forwardInput;
    }

    /// <summary>
    /// 저속 시 추가 힘 적용 (RRT 방식)
    /// </summary>
    void ApplyAdditionalForce(float forwardInput)
    {
        if (_rb.velocity.magnitude < 2f && forwardInput > 0.5f)
        {
            Vector3 forceDirection = transform.forward;
            _rb.AddForce(forceDirection * 10f, ForceMode.Acceleration);
            
            if (showDebugInfo && Time.frameCount % 60 == 0)
            {
                Debug.Log($"추가 힘 적용! 현재 속도: {_rb.velocity.magnitude:F2}");
            }
        }
    }

    /// <summary>
    /// 경로의 총 길이 계산
    /// </summary>
    float CalculatePathLength()
    {
        if (_path == null || _path.vectorPath.Count < 2) return 0f;

        float length = 0f;
        for (int i = 0; i < _path.vectorPath.Count - 1; i++)
        {
            length += Vector3.Distance(_path.vectorPath[i], _path.vectorPath[i + 1]);
        }
        return length;
    }

    void OnValidate()
    {
        // Inspector에서 값 변경 시 실시간 적용
        if (Application.isPlaying && _boatProbes != null)
        {
            ApplyMovementEnhancement();
        }
    }

    void OnDestroy()
    {
        // 원본 값 복원
        if (_boatProbes != null)
        {
            _boatProbes._enginePower = _originalEnginePower;
            _boatProbes._dragInWaterUp = _originalDragUp;
            _boatProbes._dragInWaterRight = _originalDragRight;
            _boatProbes._dragInWaterForward = _originalDragForward;
        }
    }

    void OnDrawGizmos()
    {
        if (!showDebugInfo || _path == null || _path.vectorPath.Count == 0)
            return;

        // A* 경로 그리기
        Gizmos.color = Color.green;
        for (int i = 0; i < _path.vectorPath.Count - 1; i++)
        {
            Gizmos.DrawLine(_path.vectorPath[i], _path.vectorPath[i + 1]);
            Gizmos.DrawWireSphere(_path.vectorPath[i], 0.3f);
        }

        // 현재 목표 웨이포인트
        if (_currentWaypoint < _path.vectorPath.Count)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(_path.vectorPath[_currentWaypoint], 0.6f);
            Gizmos.DrawLine(transform.position, _path.vectorPath[_currentWaypoint]);
        }

        // 목표 지점
        if (_path.vectorPath.Count > 0)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(_path.vectorPath[_path.vectorPath.Count - 1], 0.8f);
        }
    }
}