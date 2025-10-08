using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveComponent : MonoBehaviour
{
    [Header("Movement Settings")]
    [Tooltip("보트의 전진 속도입니다. 회전 시에도 이 속도를 유지합니다.")]
    public float moveSpeed = 8.0f;

    [Tooltip("보트가 방향을 전환하는 속도입니다. 낮을수록 커브가 커집니다.")]
    public float turnSpeed = 20.0f;

    [Tooltip("보트가 한 방향으로 전진할 거리입니다.")]
    public float patrolDistance = 50.0f;

    [Tooltip("보트에 따라 앞으로 가는지 뒤로 가는지")]
    public float isFront = 1.0f;

    // 비공개 변수 (내부 로직용)
    private bool isTurning = false;
    private float distanceTraveled = 0f;

    // 회전을 시작할 때의 목표 방향을 저장하는 변수
    private Quaternion turnTargetRotation;

    void Start()
    {
        // Start 함수에서는 특별히 설정할 것이 없습니다.
        // 보트의 초기 방향은 씬 에디터에서 설정한 값을 그대로 사용합니다.
    }

    void Update()
    {
        // 1. 전진: 회전 여부와 관계없이 매 프레임 전진합니다.
        transform.Translate(Vector3.forward * moveSpeed * Time.deltaTime * isFront);
        
        // 2. 상태 확인 및 방향 전환 로직
        if (isTurning)
        {
            // 목표 방향으로 부드럽게 회전합니다.
            transform.rotation = Quaternion.RotateTowards(transform.rotation, turnTargetRotation, turnSpeed * Time.deltaTime);

            // 회전이 거의 완료되면
            if (Quaternion.Angle(transform.rotation, turnTargetRotation) < 1.0f)
            {
                isTurning = false; // 직진 상태로 전환
                distanceTraveled = 0f; // 이동 거리 초기화
            }
        }
        else // '직진' 상태
        {
            distanceTraveled += moveSpeed * Time.deltaTime;
            
            // 정해진 순찰 거리를 모두 이동했다면
            if (distanceTraveled >= patrolDistance)
            {
                isTurning = true; // 회전 상태로 전환
                
                // *** 핵심 변경사항 ***
                // 바로 이 순간의 현재 방향을 기준으로 180도 반대 방향을 새로운 목표로 설정합니다.
                turnTargetRotation = transform.rotation * Quaternion.Euler(0, 180, 0);
            }
        }
    }
}