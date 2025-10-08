using UnityEngine;
using Pathfinding;
using Crest;
using RRT;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

/// <summary>
/// RRT* 경로를 정확하게 추적하는 컨트롤러 - 개선된 버전
/// </summary>
[RequireComponent(typeof(BoatProbes_AStar), typeof(AIPath))]
public class BoatAIController_RRT : MonoBehaviour
{
    [Header("RRT 경로 찾기")]
    public RRT.RRT rrtController;
    public Transform target;

    [Header("A* Movement (RRT 경로 추적용)")]
    public AIPath aiPath;
    
    [Tooltip("다음 웨이포인트까지의 거리")]
    public float nextWaypointDistance = 8f;

    [Header("Movement Control")]
    [Tooltip("목표 지점 정지 거리")]
    public float stoppingDistance = 12f;
    
    [Tooltip("감속 시작 각도")]
    public float slowDownAngle = 45f;
    
    [Tooltip("급감속 시작 각도")]
    public float sharpTurnAngle = 75f;
    
    [Tooltip("최대 전진 입력")]
    public float maxSpeedInput = 1f;
    
    [Tooltip("급회전 시 최대 속도")]
    [UnityEngine.Range(0f, 1f)]
    public float maxSpeedDuringSharpTurn = 0.4f;
    
    [Tooltip("일반 회전 시 최대 속도")]
    [UnityEngine.Range(0f, 1f)]
    public float maxSpeedDuringTurn = 0.75f;

    [Header("회전 제어 (개선됨)")]
    [Tooltip("회전 입력 배율 - 클수록 빠르게 회전")]
    [UnityEngine.Range(1f, 3f)]
    public float turnInputMultiplier = 2f;
    
    [Tooltip("최소 회전 입력값 (작은 각도에서도 회전 보장)")]
    [UnityEngine.Range(0.1f, 0.5f)]
    public float minTurnInput = 0.2f;

    [Header("예측 제어")]
    [Tooltip("앞서 볼 웨이포인트 수")]
    [UnityEngine.Range(0, 3)]
    public int lookAheadWaypoints = 1;
    
    [Tooltip("예측 거리 (m)")]
    public float lookAheadDistance = 10f;
    
    [Tooltip("속도 변화 부드러움")]
    [UnityEngine.Range(0.05f, 0.2f)]
    public float speedSmoothTime = 0.08f;

    [Header("경로 추적 정확도")]
    [Tooltip("경로 이탈 허용 거리")]
    public float maxPathDeviation = 8f;
    
    [Tooltip("웨이포인트 근접 시 감속")]
    [UnityEngine.Range(1f, 2f)]
    public float waypointSlowdownMultiplier = 1.2f;

    [Header("경로 시각화")]
    public PathToImage_Lines pathToImageGenerator;

    [Header("디버그")]
    public bool showDebugInfo = true;
    public bool showGizmos = true;
    public Color rrtPathColor = Color.green;
    public Color currentWaypointColor = Color.red;
    public Color lookAheadColor = Color.yellow;

    private float _startTime;
    private bool _isTimerRunning;
    private int _collisionCount = 0;

    private List<Vector3> _rrtPath;
    private int _currentWaypoint = 0;
    private bool _hasValidPath = false;

    private BoatProbes_AStar _boatProbes;
    private Rigidbody _rb;
    
    private float _currentSpeedLimit = 1f;
    private float _speedVelocity = 0f;

    void Awake()
    {
        _boatProbes = GetComponent<BoatProbes_AStar>();
        _rb = GetComponent<Rigidbody>();
        
        if (aiPath == null)
            aiPath = GetComponent<AIPath>();

        if (rrtController == null)
            rrtController = GetComponent<RRT.RRT>();
    }

    void Start()
    {
        if (aiPath != null)
        {
            aiPath.canMove = false;
            aiPath.canSearch = false;
        }

        if (rrtController != null)
        {
            rrtController.OnPathFound.AddListener(OnRRTPathFound);
            Debug.Log("BoatAIController_RRT: RRT 경로 찾기 이벤트 연결 완료");
        }
        else
        {
            Debug.LogError("RRT 컨트롤러를 찾을 수 없습니다!");
        }

        if (target != null)
        {
            _startTime = Time.time;
            _isTimerRunning = true;
            Debug.Log("이동 시작! RRT* 경로 탐색 중...");
        }
    }

    public void OnRRTPathFound()
    {
        Debug.Log("<color=yellow>RRT* 경로 발견! 경로를 추출합니다...</color>");

        _rrtPath = ExtractPathFromRRT();

        if (_rrtPath == null || _rrtPath.Count == 0)
        {
            Debug.LogError("RRT 경로 추출 실패!");
            return;
        }

        _currentWaypoint = 0;
        _hasValidPath = true;

        Debug.Log($"<color=cyan>RRT 경로 추출 완료! 총 {_rrtPath.Count}개의 웨이포인트</color>");
        Debug.Log($"시작: {_rrtPath[0]}, 목표: {_rrtPath[_rrtPath.Count - 1]}");
    }

    List<Vector3> ExtractPathFromRRT()
    {
        if (rrtController == null || rrtController._tree == null)
            return null;

        if (!rrtController._tree.HasFoundPath)
            return null;

        List<Vector3> path = new List<Vector3>();
        Node currentNode = rrtController._tree.TargetNode;

        if (currentNode == null)
            return null;

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
        if (collision.gameObject.CompareTag("Obstacle") || collision.gameObject.CompareTag("Boundary"))
        {
            _collisionCount++;
            Debug.Log($"<color=red>장애물과 충돌! (현재 {_collisionCount}회)</color>");
        }
    }

    void FixedUpdate()
    {
        if (!_hasValidPath || _rrtPath == null || _rrtPath.Count == 0)
        {
            _boatProbes.AI_ForwardInput = 0;
            _boatProbes.AI_TurnInput = 0;
            
            if (showDebugInfo && Time.frameCount % 120 == 0)
            {
                Debug.Log("RRT 경로 대기 중...");
            }
            return;
        }

        if (_currentWaypoint >= _rrtPath.Count)
        {
            _boatProbes.AI_ForwardInput = 0;
            _boatProbes.AI_TurnInput = 0;
            return;
        }

        Vector3 currentPos2D = new Vector3(_rb.position.x, 0, _rb.position.z);
        Vector3 waypointPos2D = new Vector3(_rrtPath[_currentWaypoint].x, 0, _rrtPath[_currentWaypoint].z);
        Vector3 targetPos2D = new Vector3(target.position.x, 0, target.position.z);

        float distanceToWaypoint = Vector3.Distance(currentPos2D, waypointPos2D);
        float distanceToTarget = Vector3.Distance(currentPos2D, targetPos2D);

        // 웨이포인트 도달 체크
        if (distanceToWaypoint < nextWaypointDistance)
        {
            _currentWaypoint++;
            Debug.Log($"<color=green>웨이포인트 {_currentWaypoint - 1} 통과! (남은: {_rrtPath.Count - _currentWaypoint})</color>");
            
            if (_currentWaypoint >= _rrtPath.Count)
            {
                Debug.Log("<color=cyan>모든 웨이포인트 통과!</color>");
                return;
            }
            
            waypointPos2D = new Vector3(_rrtPath[_currentWaypoint].x, 0, _rrtPath[_currentWaypoint].z);
            distanceToWaypoint = Vector3.Distance(currentPos2D, waypointPos2D);
        }

        // 목표 도착 확인
        if (distanceToTarget < stoppingDistance && _isTimerRunning)
        {
            _isTimerRunning = false;
            float elapsedTime = Time.time - _startTime;

            if (pathToImageGenerator != null && _rrtPath != null)
            {
                Path astarPath = ConvertRRTPathToAstarPath();
                if (astarPath != null)
                    pathToImageGenerator.GenerateAndSaveImage(astarPath);
            }

            Debug.Log($"<color=cyan>╔═══════════════════════════════╗\n" +
                      $"║   목표 지점 도착 완료!   소요 시간: {elapsedTime:F2}초     장애물 충돌: {_collisionCount}회     경로 길이: {CalculatePathLength():F2}m    웨이포인트: {_rrtPath.Count}개    </color>");

            #if UNITY_EDITOR
            EditorApplication.isPlaying = false;
            #endif
            return;
        }

        // ==========================================
        // 핵심: 개선된 제어 로직
        // ==========================================
        
        // 1. 목표 지점 계산 (예측 제어)
        Vector3 lookAheadTarget = GetLookAheadPoint();
        Vector3 desiredDirection = (lookAheadTarget - currentPos2D).normalized;

        // 2. 회전 제어 (개선됨)
        float angleToTarget = Vector3.SignedAngle(transform.forward, desiredDirection, Vector3.up);
        float absAngle = Mathf.Abs(angleToTarget);
        
        // 회전 입력 계산 - 더 강력하게
        float baseTurnInput = Mathf.Clamp(angleToTarget / 30f, -1f, 1f); // 30도 기준으로 정규화
        float turnInput = baseTurnInput * turnInputMultiplier;
        
        // 최소 회전 입력 보장 (작은 각도에서도 회전)
        if (absAngle > 5f)
        {
            float sign = Mathf.Sign(angleToTarget);
            turnInput = Mathf.Max(Mathf.Abs(turnInput), minTurnInput) * sign;
        }
        
        turnInput = Mathf.Clamp(turnInput, -1f, 1f);
        _boatProbes.AI_TurnInput = turnInput;

        // 3. 전진 제어 (개선됨)
        float targetSpeed = CalculateTargetSpeed(absAngle, distanceToTarget, distanceToWaypoint);
        
        // 부드러운 속도 변화
        _currentSpeedLimit = Mathf.SmoothDamp(
            _currentSpeedLimit, 
            targetSpeed, 
            ref _speedVelocity, 
            speedSmoothTime
        );
        
        _boatProbes.AI_ForwardInput = _currentSpeedLimit;

        // 4. 경로 이탈 모니터링
        CheckPathDeviation(currentPos2D);

        // 디버그 정보
        if (showDebugInfo && Time.frameCount % 60 == 0)
        {
            float currentSpeed = _rb.velocity.magnitude;
            Debug.Log($"WP[{_currentWaypoint}/{_rrtPath.Count}] | " +
                      $"거리: {distanceToWaypoint:F1}m | " +
                      $"각도: {angleToTarget:F1}° | " +
                      $"속도: {currentSpeed:F1}m/s ({_currentSpeedLimit:F2}) | " +
                      $"회전: {turnInput:F2}");
        }
    }

    Vector3 GetLookAheadPoint()
    {
        if (lookAheadWaypoints == 0 && lookAheadDistance == 0)
        {
            // 예측 없이 현재 웨이포인트만 사용
            return new Vector3(_rrtPath[_currentWaypoint].x, 0, _rrtPath[_currentWaypoint].z);
        }
        
        Vector3 currentPos2D = new Vector3(_rb.position.x, 0, _rb.position.z);
        float accumulatedDistance = 0f;
        
        for (int i = _currentWaypoint; i < _rrtPath.Count - 1; i++)
        {
            Vector3 wpPos = new Vector3(_rrtPath[i].x, 0, _rrtPath[i].z);
            Vector3 nextWpPos = new Vector3(_rrtPath[i + 1].x, 0, _rrtPath[i + 1].z);
            
            float segmentLength = Vector3.Distance(wpPos, nextWpPos);
            accumulatedDistance += segmentLength;
            
            if (accumulatedDistance >= lookAheadDistance || i >= _currentWaypoint + lookAheadWaypoints)
            {
                return nextWpPos;
            }
        }
        
        return new Vector3(_rrtPath[_rrtPath.Count - 1].x, 0, _rrtPath[_rrtPath.Count - 1].z);
    }

    float CalculateTargetSpeed(float absAngle, float distanceToTarget, float distanceToWaypoint)
    {
        float targetSpeed = maxSpeedInput;
        
        // 1. 각도에 따른 속도 제한 (완화됨)
        if (absAngle > sharpTurnAngle)
        {
            targetSpeed = maxSpeedDuringSharpTurn;
        }
        else if (absAngle > slowDownAngle)
        {
            float t = (absAngle - slowDownAngle) / (sharpTurnAngle - slowDownAngle);
            targetSpeed = Mathf.Lerp(maxSpeedDuringTurn, maxSpeedDuringSharpTurn, t);
        }
        else
        {
            // 작은 각도에서는 거의 최대 속도
            targetSpeed = maxSpeedInput;
        }
        
        // 2. 목표 지점 근처에서 감속
        if (distanceToTarget < stoppingDistance * 2.5f)
        {
            float distanceFactor = Mathf.Clamp01(distanceToTarget / (stoppingDistance * 2.5f));
            targetSpeed *= distanceFactor;
        }
        
        // 3. 웨이포인트 근접 시 약간 감속
        float slowdownDistance = nextWaypointDistance * waypointSlowdownMultiplier;
        if (distanceToWaypoint < slowdownDistance)
        {
            float waypointFactor = Mathf.Clamp01(distanceToWaypoint / slowdownDistance);
            targetSpeed *= Mathf.Max(waypointFactor, 0.6f); // 최소 60% 속도 유지
        }
        
        // 4. 최소 속도 보장
        return Mathf.Max(targetSpeed, 0.3f);
    }

    void CheckPathDeviation(Vector3 currentPos)
    {
        if (_currentWaypoint >= _rrtPath.Count - 1) return;
        
        Vector3 wpPos = new Vector3(_rrtPath[_currentWaypoint].x, 0, _rrtPath[_currentWaypoint].z);
        Vector3 nextWpPos = new Vector3(_rrtPath[_currentWaypoint + 1].x, 0, _rrtPath[_currentWaypoint + 1].z);
        
        Vector3 pathDirection = (nextWpPos - wpPos).normalized;
        Vector3 toPosition = currentPos - wpPos;
        float projection = Vector3.Dot(toPosition, pathDirection);
        Vector3 closestPoint = wpPos + pathDirection * Mathf.Clamp(projection, 0, Vector3.Distance(wpPos, nextWpPos));
        float deviation = Vector3.Distance(currentPos, closestPoint);
        
        if (deviation > maxPathDeviation && showDebugInfo && Time.frameCount % 120 == 0)
        {
            Debug.LogWarning($"<color=orange>⚠ 경로 이탈 경고! 편차: {deviation:F2}m (허용: {maxPathDeviation}m)</color>");
        }
    }

    Path ConvertRRTPathToAstarPath()
    {
        if (_rrtPath == null || _rrtPath.Count < 2) return null;

        var path = ABPath.Construct(_rrtPath[0], _rrtPath[_rrtPath.Count - 1], null);
        path.vectorPath = new List<Vector3>(_rrtPath);
        return path;
    }

    float CalculatePathLength()
    {
        if (_rrtPath == null || _rrtPath.Count < 2) return 0f;

        float length = 0f;
        for (int i = 0; i < _rrtPath.Count - 1; i++)
            length += Vector3.Distance(_rrtPath[i], _rrtPath[i + 1]);
        
        return length;
    }

    void OnDrawGizmos()
    {
        if (!showGizmos || _rrtPath == null || _rrtPath.Count == 0)
            return;

        Gizmos.color = rrtPathColor;
        for (int i = 0; i < _rrtPath.Count - 1; i++)
        {
            Gizmos.DrawLine(_rrtPath[i], _rrtPath[i + 1]);
        }
        
        for (int i = 0; i < _rrtPath.Count; i++)
        {
            Gizmos.color = Color.Lerp(Color.green, Color.blue, i / (float)_rrtPath.Count);
            Gizmos.DrawWireSphere(_rrtPath[i], 0.3f);
        }

        if (_hasValidPath && _currentWaypoint < _rrtPath.Count)
        {
            Gizmos.color = currentWaypointColor;
            Gizmos.DrawSphere(_rrtPath[_currentWaypoint], 0.7f);
            Gizmos.DrawLine(transform.position, _rrtPath[_currentWaypoint]);
        }

        if (_hasValidPath && Application.isPlaying)
        {
            Vector3 lookAhead = GetLookAheadPoint();
            Gizmos.color = lookAheadColor;
            Gizmos.DrawWireSphere(lookAhead, 0.9f);
            Gizmos.DrawLine(transform.position, lookAhead);
            
            Vector3 direction = (lookAhead - transform.position).normalized;
            Gizmos.DrawRay(transform.position + Vector3.up, direction * 5f);
        }

        if (_rrtPath.Count > 0)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(_rrtPath[0], 0.6f);
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(_rrtPath[_rrtPath.Count - 1], 1f);
        }
    }

    void OnDestroy()
    {
        if (rrtController != null)
            rrtController.OnPathFound.RemoveListener(OnRRTPathFound);
    }
}