using UnityEngine;
using System.Collections.Generic;

namespace RRT
{
    // 이 스크립트는 RRT 컴포넌트가 있어야 작동합니다.
    [RequireComponent(typeof(RRT))]
    public class RRTPathFollower : MonoBehaviour
    {
        [Tooltip("오브젝트의 이동 속도입니다.")]
        public float speed = 5f;

        [Header("동적 장애물 감지")]
        public LayerMask dynamicObstacleLayerMask;
        public float pathValidationDistance = 20f; // 얼마 앞까지 경로를 확인할지

        [Tooltip("장애물 레이어를 설정합니다. RRT.cs와 동일하게 설정해주세요.")]
        public LayerMask obstacleLayerMask;

        // RRT 총괄 매니저 스크립트에 대한 참조
        private RRT _rrtController;
        // RRT가 찾아낸 경로를 저장할 리스트
        private List<Vector3> _waypoints;
        // 현재 이동 목표인 웨이포인트의 인덱스
        private int _currentWaypointIndex = 0;
        // 경로를 성공적으로 추출했는지 확인하는 플래그
        private bool _hasPath = false;

        void Start()
        {
            // 같은 오브젝트에 있는 RRT 컴포넌트를 가져옵니다.
            _rrtController = GetComponent<RRT>();
        }

        void Update()
        {
            // 1. 경로가 아직 없고, RRT 컨트롤러가 경로를 찾았다면 경로를 추출합니다.
            if (!_hasPath && _rrtController != null && _rrtController._tree != null && _rrtController._tree.HasFoundPath)
            {
                ExtractPathFromTree();
            }

            // 2. 경로가 있다면, 경로를 따라 이동하고 동시에 경로 유효성을 검사합니다.
            if (_hasPath)
            {
                if (IsPathStillValid())
                {
                    MoveAlongPath();
                }
                else
                {
                     _hasPath = false; // 이동 멈춤
                    _rrtController.RequestReplanning(); // 재탐색 요청
                }
            }
        }

        // RRT 트리의 노드 연결을 따라가며 경로(웨이포인트 리스트)를 추출하는 함수
        void ExtractPathFromTree()
        {
            Debug.Log("경로를 찾았습니다! 이동을 시작합니다.");
            _waypoints = new List<Vector3>();

            // 목표 노드에서부터 시작해서 부모를 계속 따라 올라갑니다.
            Node currentNode = _rrtController._tree.TargetNode;
            while (currentNode != null)
            {
                _waypoints.Add(currentNode.Position);
                currentNode = currentNode.Parent;
            }

            // 현재 웨이포인트는 (목표 -> 시작) 순서로 저장되어 있으므로, 뒤집어줍니다.
            _waypoints.Reverse();

            _currentWaypointIndex = 0;
            _hasPath = true;
        }

        // 웨이포인트를 따라 오브젝트를 움직이는 함수
        void MoveAlongPath()
        {
            // 마지막 웨이포인트에 도달했다면 멈춥니다.
            if (_currentWaypointIndex >= _waypoints.Count)
            {
                Debug.Log("최종 목적지에 도착했습니다!");
                _hasPath = false; // 이동 중지
                return;
            }

            // 현재 목표 웨이포인트를 향해 이동합니다.
            Vector3 targetWaypoint = _waypoints[_currentWaypointIndex];
            transform.position = Vector3.MoveTowards(transform.position, targetWaypoint, speed * Time.deltaTime);

            // 현재 목표 웨이포인트에 충분히 가까워졌다면, 다음 웨이포인트를 목표로 설정합니다.
            if (Vector3.Distance(transform.position, targetWaypoint) < 0.1f)
            {
                _currentWaypointIndex++;
            }
        }

        // 현재 경로가 여전히 유효한지 검사하는 함수
        private bool IsPathStillValid()
        {
            if (_waypoints == null || _currentWaypointIndex >= _waypoints.Count - 1) return true;

            // 현재 위치에서부터 일정 거리 앞까지 경로 유효성 검사
            for (int i = _currentWaypointIndex; i < _waypoints.Count - 1; i++)
            {
                Vector3 start = _waypoints[i];
                Vector3 end = _waypoints[i + 1];

                // 너무 먼 경로는 확인하지 않음
                if (Vector3.Distance(transform.position, start) > pathValidationDistance) break;
                
                // SphereCast를 사용해 에이전트의 크기를 고려하여 경로 확인
                if (Physics.SphereCast(start, _rrtController.agentRadius, (end - start).normalized, out RaycastHit hit, Vector3.Distance(start, end), dynamicObstacleLayerMask))
                {
                    Debug.LogWarning($"경로가 동적 장애물 '{hit.transform.name}'에 의해 막혔습니다!");
                    return false;
                }
            }
            return true;
        }
        
        // RRT 컨트롤러에게 재탐색을 요청하는 함수
        private void RequestReplanning()
        {
            Debug.Log("재탐색을 요청합니다...");
            _hasPath = false; // 이동을 멈추고 다시 경로를 기다리는 상태로 전환
            _rrtController.RestartSearch(); // RRT.cs에 추가할 재탐색 함수 호출
        }
    }
}