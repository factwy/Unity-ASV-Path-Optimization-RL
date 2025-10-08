using UnityEngine;
using Pathfinding;
using Crest;

#if UNITY_EDITOR
using UnityEditor; // <-- 이 부분을 추가하세요!
#endif

[RequireComponent(typeof(BoatProbes_AStar), typeof(Seeker))]
public class BoatAIController : MonoBehaviour
{
    public Transform target; // 보트가 이동할 목표 지점

    [Header("Pathfinding")]
    public float nextWaypointDistance = 5f;
    public float pathUpdateInterval = 0.5f;

    [Header("Movement")]
    public float stoppingDistance = 10f;
    public float slowDownAngle = 45f;
    public float maxSpeedInput = 1f;

    public PathToImage_Lines pathToImageGenerator;

    // --- 타이머 및 충돌 카운터 변수 추가 ---
    private float _startTime;
    private bool _isTimerRunning;
    private int _collisionCount = 0; // 장애물 충돌 횟수를 기록할 변수
    // ------------------------------------

    private Path _path;
    private int _currentWaypoint = 0;

    private Seeker _seeker;
    private BoatProbes_AStar _boatProbes;
    private Rigidbody _rb;

    void Start()
    {
        _seeker = GetComponent<Seeker>();
        _boatProbes = GetComponent<BoatProbes_AStar>();
        _rb = GetComponent<Rigidbody>();
        InvokeRepeating(nameof(UpdatePath), 0f, pathUpdateInterval);

        if (target != null)
        {
            _startTime = Time.time;
            _isTimerRunning = true;
            Debug.Log("이동 시작! 시간 측정을 시작합니다.");
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        // 충돌한 오브젝트의 태그가 "Obstacle"인지 확인
        if (collision.gameObject.CompareTag("Obstacle"))
        {
            _collisionCount++; // 충돌 횟수 1 증가
            Debug.Log($"장애물과 충돌! (현재 {_collisionCount}회)");
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
        }
    }

    void FixedUpdate()
    {
        if (_path == null || _currentWaypoint >= _path.vectorPath.Count)
        {
            _boatProbes.AI_ForwardInput = 0;
            _boatProbes.AI_TurnInput = 0;
            return;
        }

        // --- Y축을 무시한 수평 거리 계산 (수정된 부분) ---
        Vector3 currentPosition2D = new Vector3(_rb.position.x, 0, _rb.position.z);
        Vector3 waypointPosition2D = new Vector3(_path.vectorPath[_currentWaypoint].x, 0, _path.vectorPath[_currentWaypoint].z);

        if (Vector3.Distance(currentPosition2D, waypointPosition2D) < nextWaypointDistance)
        {
            _currentWaypoint++;
            if (_currentWaypoint >= _path.vectorPath.Count) return;
            // 다음 웨이포인트의 2D 위치를 다시 계산
            waypointPosition2D = new Vector3(_path.vectorPath[_currentWaypoint].x, 0, _path.vectorPath[_currentWaypoint].z);
        }
        
        // --- 제어 로직 (Y축 무시) ---
        Vector3 waypointDirection = (waypointPosition2D - currentPosition2D).normalized;
        float distanceToTarget = Vector3.Distance(currentPosition2D, new Vector3(target.position.x, 0, target.position.z));

        // --- 목표 지점 도착 시 최종 결과 메시지 수정 ---
        if (distanceToTarget < stoppingDistance && _isTimerRunning)
        {
            _isTimerRunning = false;
            float elapsedTime = Time.time - _startTime;

            if (pathToImageGenerator != null && _path != null)
            {
                // 현재 최종 경로(_path)를 전달하여 이미지 생성
                pathToImageGenerator.GenerateAndSaveImage(_path);
            }
            
            // 최종 결과 메시지에 충돌 횟수 추가
            Debug.Log($"<color=cyan>목표 지점 도착!\n총 소요 시간: {elapsedTime.ToString("F2")}초\n장애물 충돌 횟수: {_collisionCount}회</color>");

            #if UNITY_EDITOR
                EditorApplication.isPlaying = false;
            #else
                // Application.Quit();
            #endif
        }
        

        // 1. 회전 제어
        float angleToWaypoint = Vector3.SignedAngle(transform.forward, waypointDirection, Vector3.up);
        _boatProbes.AI_TurnInput = Mathf.Clamp(angleToWaypoint / 45f, -1f, 1f);

        // 2. 전진 제어
        float forwardInput = maxSpeedInput;
        if (distanceToTarget < stoppingDistance)
        {
            forwardInput = Mathf.Clamp01(distanceToTarget / stoppingDistance);
        }
        else if (Mathf.Abs(angleToWaypoint) > slowDownAngle)
        {
            forwardInput *= Mathf.Clamp01(1f - (Mathf.Abs(angleToWaypoint) - slowDownAngle) / 90f);
        }

        _boatProbes.AI_ForwardInput = forwardInput;
    }
}