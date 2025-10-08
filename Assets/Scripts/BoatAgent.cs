using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class BoatAgent : Agent
{
    Rigidbody rBody;

    [Header("Boat Physics")]
    public float forceHeightOffset = -0.3f;
    public float enginePower = 10f;
    public float turnPower = 3f;

    [Header("References")]
    public Transform Target;
    public Transform[] boundary = new Transform[4];

    [Header("Reward Settings")]
    [Tooltip("에피소드 최대 시간 (초)")]
    public float maxEpisodeTime = 600f; // 10분
    [Tooltip("시간 보상 가중치 (대폭 증가)")]
    public float timeRewardWeight = 50.0f; // 기존 0.5f -> 5.0f
    [Tooltip("경로 효율성 보상 가중치 (대폭 증가)")]
    public float pathEfficiencyRewardWeight = 50.0f; // 기존 0.5f -> 5.0f
    [Tooltip("목표 도달 시 최종 보상")]
    public float targetReachedReward = 100.0f; // 최종 보상도 함께 증가
    [Tooltip("생존 시간에 대한 지속적인 보상")]
    public float survivalReward = 0.1f; // 새로운 보상 추가
    
    [Header("Penalty Settings")]
    [Tooltip("경계 충돌 시 패널티")]
    public float boundaryPenalty = -100f;
    [Tooltip("장애물 충돌 시 패널티")]
    public float obstaclePenalty = -0.5f;
    [Tooltip("방문했던 지역 재방문 시 패널티")]
    public float visitedPenalty = -0.001f;
    [Tooltip("제자리 회전 시 패널티")]
    public float spinningPenalty = -0.01f;
    [Tooltip("저속 주행 지속 시 패널티")]
    public float lowSpeedPenalty = -0.005f;

    [Header("Speed Control")]
    [Tooltip("이 속도 이하로 지속되면 패널티를 받습니다.")]
    public float lowSpeedThreshold = 0.5f;
    [Tooltip("저속 상태를 몇 초나 지속하면 패널티를 받을지 결정합니다.")]
    public float lowSpeedDurationThreshold = 5f;

    [Tooltip("보트 전복 시 패널티")]
    public float flippedPenalty = -50f;

    [Header("Environment References")]
    public Transform boundaryParent; // Inspector에서 Boundary 그룹 객체를 드래그앤드롭
    public Transform obstacleParent; // Inspector에서 Obstacle 그룹 객체를 드래그앤드롭

    private List<Transform> boundaryParts = new List<Transform>();
    private List<Transform> obstacleParts = new List<Transform>();

    // 최대 관측할 객체 수 (⚠️ 중요: 이 숫자는 고정되어야 합니다)
    private const int MAX_BOUNDARY_PARTS = 4;
    private const int MAX_OBSTACLES = 13;
    
    // 내부 로직용 변수
    private float episodeStartTime;
    private float totalDistanceTraveled;
    private Vector3 lastPosition;
    private HashSet<Vector2Int> visitedGridCells;
    private int gridCellSize = 5; // 5x5미터 크기의 그리드 셀
    private float lowSpeedTimer;
    private float initialEuclideanDistance;
    private float initialManhattanDistance;

    protected override void Awake()
    {
        rBody = GetComponent<Rigidbody>(); // 기존 Start()에 있던 내용

        // Boundary 자식 객체들의 Transform 정보 수집
        foreach (Transform part in boundaryParent)
        {
            if (boundaryParts.Count >= MAX_BOUNDARY_PARTS) break; // 최대 개수 초과 방지
            boundaryParts.Add(part);
        }

        // Obstacle 자식 객체들의 Transform 정보 수집
        foreach (Transform part in obstacleParent)
        {
            if (obstacleParts.Count >= MAX_OBSTACLES) break; // 최대 개수 초과 방지
            obstacleParts.Add(part);
        }
    }

    public override void OnEpisodeBegin()
    {
        // --- 에이전트 및 타겟 위치 초기화 ---
        rBody.angularVelocity = Vector3.zero;
        rBody.velocity = Vector3.zero;
        transform.localPosition = new Vector3(-68.0f, 1.0f, -68.0f);
        transform.localRotation = Quaternion.Euler(0, 0, 0);

        Target.localPosition = new Vector3(80.0f + Random.value * 10.0f, 1.0f, 80.0f + Random.value * 10.0f);

        // --- 보상 계산을 위한 변수 초기화 ---
        episodeStartTime = Time.time;
        totalDistanceTraveled = 0f;
        lastPosition = transform.localPosition;
        lowSpeedTimer = 0f;

        // 방문 기록 초기화
        visitedGridCells = new HashSet<Vector2Int>();
        RecordVisitedPosition();

        // 초기 거리 계산
        initialEuclideanDistance = Vector3.Distance(transform.localPosition, Target.localPosition);
        initialManhattanDistance = Mathf.Abs(transform.localPosition.x - Target.localPosition.x) + Mathf.Abs(transform.localPosition.z - Target.localPosition.z);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // --- 기존 관측 정보 (총 12개) ---
        // 1. 보트의 속력 (1개)
        sensor.AddObservation(rBody.velocity.magnitude);
        // 2. 보트의 현재 회전값 (4개)
        sensor.AddObservation(transform.rotation);
        // 3. 보트의 현재 위치 (3개)
        sensor.AddObservation(transform.localPosition);
        // 4. 타겟의 현재 위치 (3개)
        sensor.AddObservation(Target.localPosition);
        // 5. 보트와 타겟 사이의 거리 (1개)
        float distanceToTarget = Vector3.Distance(transform.localPosition, Target.localPosition);
        sensor.AddObservation(distanceToTarget);

        // --- 새로운 관측 정보: 경계(Boundary) 객체들 ---
        for (int i = 0; i < MAX_BOUNDARY_PARTS; i++)
        {
            if (i < boundaryParts.Count)
            {
                // 에이전트 기준 상대 위치 (Vector3 = 3개)
                sensor.AddObservation(transform.InverseTransformPoint(boundaryParts[i].position));
                // 객체의 크기 (Vector3 = 3개)
                sensor.AddObservation(boundaryParts[i].localScale);
            }
            else
            {
                // 객체 수가 부족하면 0으로 채워넣기 (Padding)
                sensor.AddObservation(Vector3.zero); // 위치 정보
                sensor.AddObservation(Vector3.zero); // 크기 정보
            }
        }

        // --- 새로운 관측 정보: 장애물(Obstacle) 객체들 ---
        for (int i = 0; i < MAX_OBSTACLES; i++)
        {
            if (i < obstacleParts.Count)
            {
                // 에이전트 기준 상대 위치 (Vector3 = 3개)
                sensor.AddObservation(transform.InverseTransformPoint(obstacleParts[i].position));
                // 객체의 크기 (Vector3 = 3개)
                sensor.AddObservation(obstacleParts[i].localScale);
            }
            else
            {
                // 객체 수가 부족하면 0으로 채워넣기 (Padding)
                sensor.AddObservation(Vector3.zero); // 위치 정보
                sensor.AddObservation(Vector3.zero); // 크기 정보
            }
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // --- 행동 실행 ---
        float throttle = actionBuffers.ContinuousActions[1];
        float steer = actionBuffers.ContinuousActions[0];

        var forcePosition = rBody.worldCenterOfMass + transform.up * forceHeightOffset;
        rBody.AddForceAtPosition(transform.forward * enginePower * throttle, forcePosition, ForceMode.Acceleration);
        rBody.AddTorque(transform.up * turnPower * steer, ForceMode.Acceleration);

        // --- 지속적인 보상 및 패널티 계산 ---
        // 1. 생존 보상 추가
        AddReward(survivalReward * Time.fixedDeltaTime);

        // 2. 이동 거리 누적
        totalDistanceTraveled += Vector3.Distance(transform.localPosition, lastPosition);
        lastPosition = transform.localPosition;

        // 3. 방문 지역 패널티
        ApplyVisitedPenalty();

        // 4. 제자리 회전 패널티
        if (Mathf.Abs(rBody.angularVelocity.y) > 0.5f && rBody.velocity.magnitude < lowSpeedThreshold)
        {
            AddReward(spinningPenalty * Time.fixedDeltaTime);
        }

        // 5. 저속 지속 패널티
        if (rBody.velocity.magnitude < lowSpeedThreshold)
        {
            lowSpeedTimer += Time.fixedDeltaTime;
            if (lowSpeedTimer > lowSpeedDurationThreshold)
            {
                AddReward(lowSpeedPenalty * Time.fixedDeltaTime);
            }
        }
        else
        {
            lowSpeedTimer = 0f; // 속도가 기준 이상이면 타이머 리셋
        }

        // 6. 보트가 뒤집힌 경우
        if (Vector3.Dot(transform.up, Vector3.up) < 0f)
        {
            // 전복 패널티를 부여하고
            AddReward(flippedPenalty);
            // 에피소드를 즉시 종료합니다.
            EndEpisode();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        // Obstacle 태그와 충돌하면 중량의 패널티 부여
        if (collision.gameObject.CompareTag("Obstacle"))
        {
            AddReward(obstaclePenalty);
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        // Target과 충돌하면 성공
        if (other.transform == Target)
        {
            // 1. 시간에 대한 보상
            float elapsedTime = Time.time - episodeStartTime;
            float timeReward = (maxEpisodeTime - elapsedTime) / maxEpisodeTime * timeRewardWeight;
            AddReward(timeReward);

            // 2. 경로 효율성에 대한 보상/패널티
            if (totalDistanceTraveled >= initialEuclideanDistance && totalDistanceTraveled < initialManhattanDistance)
            {
                AddReward(pathEfficiencyRewardWeight); // 중량의 리워드
            }
            else if (totalDistanceTraveled >= initialManhattanDistance && totalDistanceTraveled < initialManhattanDistance * 2)
            {
                AddReward(pathEfficiencyRewardWeight * 0.5f); // 소량의 리워드
            }
            else
            {
                AddReward(-pathEfficiencyRewardWeight); // 패널티
            }

            // 최종 성공 보상
            AddReward(targetReachedReward);
            EndEpisode();
        }

        // Boundary와 충돌하면 실패
        foreach (Transform b in boundary)
        {
            if (other.transform == b)
            {
                AddReward(boundaryPenalty);
                EndEpisode();
                break;
                // AddReward(obstaclePenalty);
            }
        }
    }
    
    // --- 방문 기록 관련 함수들 ---
    void RecordVisitedPosition()
    {
        Vector2Int gridCell = WorldToGridCell(transform.localPosition);
        visitedGridCells.Add(gridCell);
    }

    void ApplyVisitedPenalty()
    {
        Vector2Int currentGridCell = WorldToGridCell(transform.localPosition);
        if (visitedGridCells.Contains(currentGridCell))
        {
            // 이미 방문한 셀이라면 패널티
            AddReward(visitedPenalty);
        }
        else
        {
            // 새로운 셀이라면 기록
            visitedGridCells.Add(currentGridCell);
        }
    }

    Vector2Int WorldToGridCell(Vector3 position)
    {
        int x = Mathf.FloorToInt(position.x / gridCellSize);
        int z = Mathf.FloorToInt(position.z / gridCellSize);
        return new Vector2Int(x, z);
    }
    
    // for Test
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }
}