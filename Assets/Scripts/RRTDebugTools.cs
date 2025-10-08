using UnityEngine;
using RRT;

/// <summary>
/// RRT가 경로를 찾지 못하는 이유를 진단하는 도구
/// </summary>
public class RRTDebugTool : MonoBehaviour
{
    public RRT.RRT rrtController;
    public Transform target;
    public GameObject boat;
    
    [Header("디버그 옵션")]
    public bool showDebugSpheres = true;
    public bool checkEveryFrame = true;
    public float sphereSize = 2f;

    private bool hasChecked = false;

    void Start()
    {
        if (rrtController == null)
            rrtController = FindObjectOfType<RRT.RRT>();
        
        if (target == null && rrtController != null)
            target = rrtController.target.transform;
        
        if (boat == null)
            boat = GameObject.FindGameObjectWithTag("Player");

        Invoke(nameof(PerformDiagnostics), 1f);
    }

    void Update()
    {
        if (checkEveryFrame && Time.frameCount % 120 == 0)
        {
            PerformDiagnostics();
        }
    }

    [ContextMenu("진단 실행")]
    void PerformDiagnostics()
    {
        if (hasChecked && !checkEveryFrame) return;
        hasChecked = true;

        Debug.Log("========== RRT 진단 시작 ==========");

        // 1. 기본 설정 확인
        CheckBasicSettings();

        // 2. 거리 확인
        CheckDistances();

        // 3. 탐색 공간 확인
        CheckSearchSpace();

        // 4. 장애물 확인
        CheckObstacles();

        // 5. Agent Radius 확인
        CheckAgentRadius();

        // 6. 목표 도달 가능성 확인
        CheckTargetReachability();

        Debug.Log("========== RRT 진단 완료 ==========");
    }

    void CheckBasicSettings()
    {
        Debug.Log("--- 기본 설정 확인 ---");

        if (rrtController == null)
        {
            Debug.LogError("❌ RRT 컨트롤러가 없습니다!");
            return;
        }

        Debug.Log($"✓ RRT Strategy: {rrtController.rrtStrategy}");
        Debug.Log($"✓ Iterate Automatically: {rrtController.iterateAutomatically}");
        Debug.Log($"✓ Max Iterations: {rrtController.maxIterations}");
        Debug.Log($"✓ Max Step Size: {rrtController.maxStepSize}");
        Debug.Log($"✓ Agent Radius: {rrtController.agentRadius}");

        if (target == null)
        {
            Debug.LogError("❌ Target이 설정되지 않았습니다!");
        }
        else
        {
            Debug.Log($"✓ Target: {target.name} at {target.position}");
        }

        if (rrtController._tree != null)
        {
            Debug.Log($"✓ Tree Root: {rrtController._tree.RootNode.Position}");
            Debug.Log($"✓ Has Found Path: {rrtController._tree.HasFoundPath}");
        }
    }

    void CheckDistances()
    {
        Debug.Log("--- 거리 확인 ---");

        if (boat == null || target == null)
        {
            Debug.LogError("❌ Boat 또는 Target이 없습니다!");
            return;
        }

        Vector3 startPos = boat.transform.position;
        Vector3 targetPos = target.position;
        float distance = Vector3.Distance(startPos, targetPos);

        Debug.Log($"시작 위치: {startPos}");
        Debug.Log($"목표 위치: {targetPos}");
        Debug.Log($"직선 거리: {distance:F2}m");

        // MaxStepSize와 비교
        if (rrtController != null)
        {
            float stepsNeeded = distance / rrtController.maxStepSize;
            Debug.Log($"필요한 최소 스텝 수: {stepsNeeded:F0}");

            if (stepsNeeded > rrtController.maxIterations * 0.5f)
            {
                Debug.LogWarning($"⚠️ Max Iterations ({rrtController.maxIterations})가 부족할 수 있습니다!");
                Debug.LogWarning($"   권장: {Mathf.CeilToInt(stepsNeeded * 3)} 이상");
            }
        }
    }

    void CheckSearchSpace()
    {
        Debug.Log("--- 탐색 공간 확인 ---");

        if (rrtController == null) return;

        Vector3 min = rrtController.minimum;
        Vector3 max = rrtController.maximum;
        Vector3 size = max - min;

        Debug.Log($"탐색 공간 최소: {min}");
        Debug.Log($"탐색 공간 최대: {max}");
        Debug.Log($"탐색 공간 크기: {size}");

        // 시작점과 목표점이 탐색 공간 내에 있는지 확인
        if (boat != null)
        {
            Vector3 startPos = boat.transform.position;
            bool startInBounds = IsInBounds(startPos, min, max);
            
            if (!startInBounds)
            {
                Debug.LogError($"❌ 시작 위치 {startPos}가 탐색 공간 밖입니다!");
                Debug.LogError($"   X: {startPos.x} (범위: {min.x} ~ {max.x})");
                Debug.LogError($"   Y: {startPos.y} (범위: {min.y} ~ {max.y})");
                Debug.LogError($"   Z: {startPos.z} (범위: {min.z} ~ {max.z})");
            }
            else
            {
                Debug.Log($"✓ 시작 위치가 탐색 공간 내에 있습니다.");
            }
        }

        if (target != null)
        {
            Vector3 targetPos = target.position;
            bool targetInBounds = IsInBounds(targetPos, min, max);
            
            if (!targetInBounds)
            {
                Debug.LogError($"❌ 목표 위치 {targetPos}가 탐색 공간 밖입니다!");
                Debug.LogError($"   X: {targetPos.x} (범위: {min.x} ~ {max.x})");
                Debug.LogError($"   Y: {targetPos.y} (범위: {min.y} ~ {max.y})");
                Debug.LogError($"   Z: {targetPos.z} (범위: {min.z} ~ {max.z})");
                
                // 권장 범위 계산
                float padding = 20f;
                Vector3 suggestedMin = new Vector3(
                    Mathf.Min(boat.transform.position.x, targetPos.x) - padding,
                    Mathf.Min(boat.transform.position.y, targetPos.y) - padding,
                    Mathf.Min(boat.transform.position.z, targetPos.z) - padding
                );
                Vector3 suggestedMax = new Vector3(
                    Mathf.Max(boat.transform.position.x, targetPos.x) + padding,
                    Mathf.Max(boat.transform.position.y, targetPos.y) + padding,
                    Mathf.Max(boat.transform.position.z, targetPos.z) + padding
                );
                
                Debug.LogWarning($"💡 권장 탐색 공간:");
                Debug.LogWarning($"   Minimum: {suggestedMin}");
                Debug.LogWarning($"   Maximum: {suggestedMax}");
            }
            else
            {
                Debug.Log($"✓ 목표 위치가 탐색 공간 내에 있습니다.");
            }
        }
    }

    void CheckObstacles()
    {
        Debug.Log("--- 장애물 확인 ---");

        if (rrtController == null) return;

        LayerMask obstacleMask = rrtController.obstacleLayerMask;
        Debug.Log($"Obstacle Layer Mask: {obstacleMask.value}");

        if (obstacleMask.value == 0)
        {
            Debug.LogWarning("⚠️ Obstacle Layer Mask가 Nothing입니다. 장애물이 없나요?");
        }

        // 시작점과 목표점 사이에 장애물이 있는지 확인
        if (boat != null && target != null)
        {
            Vector3 start = boat.transform.position;
            Vector3 end = target.position;
            Vector3 direction = end - start;
            float distance = direction.magnitude;

            RaycastHit hit;
            if (Physics.Raycast(start, direction.normalized, out hit, distance, obstacleMask))
            {
                Debug.LogWarning($"⚠️ 시작점과 목표점 사이에 장애물이 있습니다!");
                Debug.LogWarning($"   장애물: {hit.collider.gameObject.name}");
                Debug.LogWarning($"   거리: {hit.distance:F2}m");
            }
            else
            {
                Debug.Log($"✓ 직선 경로에 장애물이 없습니다.");
            }
        }
    }

    void CheckAgentRadius()
    {
        Debug.Log("--- Agent Radius 확인 ---");

        if (rrtController == null) return;

        float agentRadius = rrtController.agentRadius;
        Debug.Log($"Agent Radius: {agentRadius}");

        // 보트의 실제 크기와 비교
        if (boat != null)
        {
            Collider boatCollider = boat.GetComponent<Collider>();
            if (boatCollider != null)
            {
                Bounds bounds = boatCollider.bounds;
                float maxSize = Mathf.Max(bounds.size.x, bounds.size.z) / 2f;
                
                Debug.Log($"보트 실제 반지름 (대략): {maxSize:F2}");
                
                if (agentRadius < maxSize)
                {
                    Debug.LogWarning($"⚠️ Agent Radius가 보트보다 작습니다!");
                    Debug.LogWarning($"   권장: {maxSize * 1.2f:F2} 이상");
                }
                else if (agentRadius > maxSize * 3f)
                {
                    Debug.LogWarning($"⚠️ Agent Radius가 너무 큽니다! 경로를 찾기 어려울 수 있습니다.");
                    Debug.LogWarning($"   권장: {maxSize * 1.5f:F2} 정도");
                }
                else
                {
                    Debug.Log($"✓ Agent Radius가 적절합니다.");
                }
            }
        }
    }

    void CheckTargetReachability()
    {
        Debug.Log("--- 목표 도달 가능성 확인 ---");

        if (rrtController == null || boat == null || target == null) return;

        // 목표 주변에 장애물이 있는지 확인
        Vector3 targetPos = target.position;
        float checkRadius = rrtController.agentRadius * 2f;
        
        Collider[] nearbyObstacles = Physics.OverlapSphere(
            targetPos, 
            checkRadius, 
            rrtController.obstacleLayerMask
        );

        if (nearbyObstacles.Length > 0)
        {
            Debug.LogError($"❌ 목표 주변 {checkRadius}m 내에 {nearbyObstacles.Length}개의 장애물이 있습니다!");
            foreach (var obs in nearbyObstacles)
            {
                float dist = Vector3.Distance(targetPos, obs.transform.position);
                Debug.LogError($"   - {obs.gameObject.name} (거리: {dist:F2}m)");
            }
            Debug.LogError("   목표가 장애물로 둘러싸여 있을 수 있습니다!");
        }
        else
        {
            Debug.Log($"✓ 목표 주변에 장애물이 없습니다.");
        }

        // MaxStepSize로 목표에 도달 가능한지 확인
        float maxStepSize = rrtController.maxStepSize;
        float distanceToTarget = Vector3.Distance(boat.transform.position, targetPos);
        
        Debug.Log($"Max Step Size: {maxStepSize}");
        Debug.Log($"목표까지 거리: {distanceToTarget:F2}m");
        
        if (maxStepSize < 1f)
        {
            Debug.LogWarning($"⚠️ Max Step Size가 너무 작습니다! 경로 찾기가 느릴 수 있습니다.");
            Debug.LogWarning($"   권장: 3-7 정도");
        }
    }

    bool IsInBounds(Vector3 pos, Vector3 min, Vector3 max)
    {
        return pos.x >= min.x && pos.x <= max.x &&
               pos.y >= min.y && pos.y <= max.y &&
               pos.z >= min.z && pos.z <= max.z;
    }

    void OnDrawGizmos()
    {
        if (!showDebugSpheres || rrtController == null) return;

        // 탐색 공간 그리기
        Gizmos.color = Color.yellow;
        Vector3 center = (rrtController.minimum + rrtController.maximum) / 2f;
        Vector3 size = rrtController.maximum - rrtController.minimum;
        Gizmos.DrawWireCube(center, size);

        // 시작점
        if (boat != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(boat.transform.position, sphereSize);
        }

        // 목표점
        if (target != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(target.position, sphereSize);
            
            // Agent Radius 표시
            Gizmos.color = new Color(1, 0, 0, 0.3f);
            Gizmos.DrawWireSphere(target.position, rrtController.agentRadius);
        }

        // 직선 경로
        if (boat != null && target != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(boat.transform.position, target.position);
        }
    }
}