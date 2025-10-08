﻿using System;
using RRT.RRTStrategies;
using UnityEngine;
using UnityEngine.Events;

namespace RRT
{
    public class RRT : MonoBehaviour
    {
        /// <summary>
        ///     Enum representing the available RRT strategies. Used to enable changing the strategy in the unity inspector
        /// </summary>
        public enum RRTStrategyEnum
        {
            RRTBasic,
            RRTInformed,
            RRTStar,
            RRTStarInformed,
            RRTStarInformedPruning,
            RRTStarInformedPruningEllipse,
            RRTInformedPlanar,
            RRTInformedPlanarWithReduction,
            RRTInformedWithReduction
        }

        /// <summary>
        ///     Clicking this checkbox will delete the current search tree and restart the search with the set preferences and
        ///     search algorithm.
        /// </summary>
        [Tooltip(
            "Clicking this checkbox will delete the current search tree and restart the search with the set preferences and search algorithm.")]
        [Header("Control RRT execution")]
        public bool restartRRT;

        /// <summary>
        ///     This is a variable which acts as a button click in the Inspector.
        ///     Checking the checkbox will result in adding one new node to the RRT.
        ///     This only works if the IterateAutomatically checkbox is unchecked
        /// </summary>
        [Tooltip("Checking the checkbox will result in attempting to add one new node to the RRT." +
                 " This only works if the IterateAutomatically checkbox is unchecked")]
        public bool makeOneIteration;

        /// <summary>
        ///     If this bool is true, the RRT will be generated automatically every frame.
        ///     This needs to be false, so that MakeOneIteration takes effect.
        /// </summary>
        [Tooltip("If this checkbox is checked, the RRT will be generated automatically every frame." +
                 "This needs to be deactivated, so that makeOneIteration takes effect.")]
        public bool iterateAutomatically;

        /// <summary>
        ///     Display the rrt tree in the scene when true
        /// </summary>
        [Header("Visualization")]
        [Tooltip("Displays the rrt tree in the scene when enabled")]
        public bool displayRRTAsGizmo = true;

        /// <summary>
        ///     Display only the found path when true
        /// </summary>
        [Tooltip("Displays only the found path in the scene when enabled." +
                 "For this option to take action, 'Display RRT As Gizmo' needs to be enabled.")]
        public bool displayFoundPathOnly;

        [Header("Agent Parameters")]
        [Tooltip("에이전트(보트)의 반지름입니다. 충돌 검사에 사용됩니다.")]
        public float agentRadius = 1.0f;

        /// <summary>
        ///     The game object the RRT algorithm should search for
        /// </summary>
        [Header("General Parameters")] public GameObject target;

        /// <summary>
        ///     Choose the RRT search algorithm that should be used to search the path
        /// </summary>
        [Tooltip(
            "Select the RRT search algorithm that should be used to search a collision free path." +
            " To apply the change, click 'restart search' or move the target.")]
        public RRTStrategyEnum rrtStrategy = RRTStrategyEnum.RRTBasic;

        /// <summary>
        ///     The position of the edge of the search area with the minimum values
        /// </summary>
        [Tooltip("The position of the edge of the search area with the minimum values")]
        public Vector3 minimum = new Vector3(-10, -10, -5);

        /// <summary>
        ///     The position the edge of the search area with the maximum values
        /// </summary>
        [Tooltip("The position the edge of the search area with the maximum values")]
        public Vector3 maximum = new Vector3(10, 10, 5);

        /// <summary>
        ///     The maximum distance one Node can have from the next node
        /// </summary>
        [Tooltip(
            "The maximum distance one node can have from the closest next node, also called the maximum branch length")]
        public float maxStepSize = 1;

        /// <summary>
        ///     Maximum iterations the RRT search will perform to find the goal.
        ///     If the MaxIterations is exceeded, the search will be stopped.
        /// </summary>
        [Tooltip("Maximum amount of nodes the RRT search will add to the search tree." +
                 "If the MaxIterations is exceeded, the search will be stopped, even if no path was found." +
                 " This does not mean, that the search tree holds maxIterations nodes at this point, the search only tried to add maxIterations of nodes to the tree.")]
        [Range(1000, 100000)]
        public int maxIterations = 5000;

        /// <summary>
        ///     Maximus amount of new Nodes which will be added each frame update
        /// </summary>
        [Tooltip(
            "The maximum amount of nodes which will be added to the RRT search tree within one fixed update cycle.")]
        public int maxNodesPerFixedUpdate = 20;

        /// <summary>
        ///     Layer which contains all the obstacles the RRT should avoid
        /// </summary>
        [Tooltip("Layer which contains all the obstacles the RRT search should avoid")]
        public LayerMask obstacleLayerMask;

        /// <summary>
        ///     Defines how often the informed RRT search should place the random point at the target position.
        ///     So if it is set to 10 every 10th time the random point won't be random but places at the target location.
        ///     This value should be >= 1. When the value is one, the random point will always be places at the target location.
        ///     Changing thin value can have a huge impact on the runtime and the path quality.
        /// </summary>
        [Tooltip("Defines how often the informed RRT search should place the random point at the target position." +
                 "So if it is set to 10 every 10th time the random point won't be random but places at the target location." +
                 "When the value is one, the random point will always be places at the target location." +
                 "Changing this value can have a huge impact on the runtime and the path quality.")]
        [Range(1, 50)]
        public int targetBias = 10;

        /// <summary>
        ///     Radius in which the RRT star is restructured around a newly added node.
        /// </summary>
        [Tooltip("Radius in which the RRT star is restructured around a newly added node")]
        [Range(0.1f, 100)]
        public float radiusRRTStar = 1;

        /// <summary>
        ///     Influences the amount of nodes which should be created in each horizontal and vertical plane when using the
        ///     improved RRT search strategy.
        ///     Increasing the in plane exploration factor, makes the search strategy search longer in the vertical and horizontal
        ///     plane for a possible trajectory, before trying the 3d search.
        /// </summary>
        [Tooltip(
            "Influences the amount of nodes which should be created in each horizontal and vertical plane when using the planar RRT search strategy." +
            "Increasing the in plane exploration factor, makes the search strategy search longer in the vertical and horizontal plane for a possible path, before trying the 3d search.")]
        [Range(5, 200)]
        public float inPlaneExplorationFactor = 15;

        /// <summary>
        ///     Used for RRT strategies with reduction.
        ///     This is the limit of how often trying to add a child node to a node can fail, before removing the node from the
        ///     search tree.
        ///     Increasing the value causes nodes to be deleted from the tree later, since more failed attempts are allowed.
        /// </summary>
        [Tooltip(
            "Used for RRT strategies with reduction. " +
            "This is the limit of how often trying to add a child node to a node can fail, before removing the node from the search tree." +
            "Increasing the value causes nodes to be deleted from the tree later, since more failed attempts are allowed.")]
        [Range(1, 20)]
        public int maxAllowedFailuresPerNode = 5;

        /// <summary>
        ///     If the RRT should only explore 2 d space, in this case, this means x and y direction
        /// </summary>
        [Header("Parameters for 2D RRT")]
        [Tooltip(
            "Check if the RRT should only explore 2D space, in this case, this means x and y direction. If unchecked, the search is performed in 3D space.")]
        public bool onlySearch2DSpace;

        [Header("Events")]
        public UnityEvent OnPathFound; // 경로를 찾았을 때 호출할 이벤트

        [Header("Path Finding Control")]
        [Tooltip("경로를 찾은 후에도 계속 최적화를 진행할지 여부")]
        public bool continueOptimizationAfterPath = false;

        private bool _continueAfterFoundPath;

        private int _counter;
        private RRTSearchStrategy _rrtSearchStrategy;

        /// <summary>
        ///     RRT tree which holds all the visited randomly created nodes
        /// </summary>
        public Tree _tree;

        public void RestartSearch()
        {
            InitTreeAndRRTStrategy();
        }

        private void Start()
        {
            InitTreeAndRRTStrategy();
        }

        private void FixedUpdate()
        {
            if (restartRRT) InitTreeAndRRTStrategy();

            if (_tree == null || _rrtSearchStrategy == null)
                return;

            if (_counter >= maxIterations)
            {
                if (_counter == maxIterations)
                {
                    Debug.Log($"최대 반복 횟수 {maxIterations} 도달. RRT 실행 중지");
                    _counter++; // 메시지가 한 번만 출력되도록
                }
                return;
            }

            // 경로를 찾았고 추가 최적화가 필요없으면 중지
            if (_tree.HasFoundPath && !_continueAfterFoundPath)
            {
                return;
            }

            if (!iterateAutomatically)
            {
                if (makeOneIteration)
                {
                    makeOneIteration = false;
                    var newNode = _rrtSearchStrategy.AddOneNodeToTree();
                    if (newNode != null)
                    {
                        _counter++;
                    }
                }
            }
            else
            {
                var i = 0;
                while (i < maxNodesPerFixedUpdate && _counter < maxIterations &&
                       (!_tree.HasFoundPath || _continueAfterFoundPath))
                {
                    var newNode = _rrtSearchStrategy.AddOneNodeToTree();
                    if (newNode != null)
                    {
                        _counter++;
                    }
                    i++;
                }
            }
        }

        public void RequestReplanning()
        {
            if (_tree != null && _tree.HasFoundPath)
            {
                Debug.Log("재탐색 요청 수신. 탐색을 다시 시작합니다.");
                InitTreeAndRRTStrategy();
            }
        }

        /// <summary>
        ///     Draw the RRT as gizmo
        /// </summary>
        private void OnDrawGizmos()
        {
            if (!displayRRTAsGizmo || _tree == null) return;

            if (!displayFoundPathOnly)
            {
                //draw the whole search tree
                var lines = _tree.RootNode.GetLinesToChildren();
                Gizmos.color = Color.blue;
                foreach (var (start, end) in lines) Gizmos.DrawLine(start, end);
            }

            if (!_tree.HasFoundPath) return;
            var node = _tree.TargetNode;
            var nextNode = node.Parent;
            while (nextNode != null)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(node.Position, nextNode.Position);
                node = nextNode;
                nextNode = node.Parent;
            }
        }

        /// <summary>
        ///     Sets the _tree to an new tree which only contains the start position.
        ///     It also sets the _rrtSearchStrategy to the one selected in the UI, using the RRTStrategyEnum
        /// </summary>
        private void InitTreeAndRRTStrategy()
        {
            _continueAfterFoundPath = continueOptimizationAfterPath;
            restartRRT = false;
            _counter = 0;
            _tree = new Tree(transform.position, this);
            var rrtConfig = new RRTConfig(_tree, maximum, minimum, obstacleLayerMask, target, maxStepSize,
                onlySearch2DSpace, agentRadius);
            
            //Select the RRT Search Strategy
            switch (rrtStrategy)
            {
                case RRTStrategyEnum.RRTBasic:
                    _rrtSearchStrategy = new RRTBasic(rrtConfig);
                    break;
                case RRTStrategyEnum.RRTStar:
                    _rrtSearchStrategy = new RRTStar(rrtConfig, radiusRRTStar);
                    break;
                case RRTStrategyEnum.RRTStarInformed:
                    _rrtSearchStrategy = new RRTStarInformed(rrtConfig, radiusRRTStar, targetBias);
                    _continueAfterFoundPath = true;
                    break;
                case RRTStrategyEnum.RRTStarInformedPruning:
                    _rrtSearchStrategy = new RRTStarInformedPruning(rrtConfig, radiusRRTStar, targetBias);
                    _continueAfterFoundPath = true;
                    break;
                case RRTStrategyEnum.RRTStarInformedPruningEllipse:
                    _rrtSearchStrategy = new RRTStarInformedPruningEllipse(rrtConfig, radiusRRTStar, targetBias);
                    _continueAfterFoundPath = true;
                    break;
                case RRTStrategyEnum.RRTInformedPlanar:
                    _rrtSearchStrategy = new RRTInformedPlanar(rrtConfig, targetBias, inPlaneExplorationFactor);
                    break;
                case RRTStrategyEnum.RRTInformed:
                    _rrtSearchStrategy = new RRTInformed(rrtConfig, targetBias);
                    break;
                case RRTStrategyEnum.RRTInformedWithReduction:
                    _rrtSearchStrategy = new RRTInformedWithReduction(rrtConfig, targetBias, maxAllowedFailuresPerNode);
                    break;
                case RRTStrategyEnum.RRTInformedPlanarWithReduction:
                    _rrtSearchStrategy = new RRTInformedPlanarWithReduction(rrtConfig, targetBias,
                        inPlaneExplorationFactor, maxAllowedFailuresPerNode);
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
            
            Debug.Log($"RRT 초기화 완료 - 전략: {rrtStrategy}, 최적화 계속: {_continueAfterFoundPath}");
        }
        
        // 경로를 찾았을 때 호출되는 함수
        public void NotifyPathFound()
        {
            Debug.Log($"경로 발견! (노드 수: {_counter}, 최적화 계속: {_continueAfterFoundPath})");
            
            // 이벤트 호출 - RRTBoatAI가 경로를 추출할 수 있도록
            if (OnPathFound != null)
            {
                OnPathFound.Invoke();
            }
            else
            {
                Debug.LogWarning("OnPathFound 이벤트에 리스너가 연결되지 않았습니다!");
            }
        }
    }
}