using UnityEngine;
using Pathfinding;
using System.IO;

public class PathToImage_Lines : MonoBehaviour
{
    [Header("이미지 설정")]
    public int textureSize = 1024; // 정사각형 텍스처

    [Header("색상 및 두께")]
    public Color backgroundColor = Color.white;
    public Color unwalkableColor = new Color(0.2f, 0.2f, 0.2f);
    public Color connectionLineColor = new Color(0, 0, 1, 0.5f);
    public Color pathLineColor = Color.red;
    [Range(1, 5)]
    public int pathLineThickness = 3;

    public void GenerateAndSaveImage(Pathfinding.Path p)
    {
        var graph = AstarPath.active.data.gridGraph;
        if (graph == null)
        {
            Debug.LogError("Grid Graph를 찾을 수 없습니다!");
            return;
        }

        Texture2D texture = new Texture2D(textureSize, textureSize);
        Color[] pixels = new Color[textureSize * textureSize];
        for (int i = 0; i < pixels.Length; i++) pixels[i] = backgroundColor;
        texture.SetPixels(pixels);

        // 2. 장애물(Unwalkable) 영역 그리기
        for (int i = 0; i < graph.nodes.Length; i++)
        {
            GridNode node = graph.nodes[i] as GridNode;
            if (!node.Walkable)
            {
                // --- 수정된 부분 1 ---
                Vector2Int pixelCoords = WorldToPixel((Vector3)node.position, graph);
                texture.SetPixel(pixelCoords.x, pixelCoords.y, unwalkableColor);
            }
        }

        // 3. 모든 '갈 수 있는' 노드 연결선 그리기 (파란 가지)
        graph.GetNodes(node => {
            if (node.Walkable)
            {
                // --- 수정된 부분 2 ---
                Vector2Int startPixel = WorldToPixel((Vector3)node.position, graph);
                node.GetConnections(other => {
                    // --- 수정된 부분 3 ---
                    Vector2Int endPixel = WorldToPixel((Vector3)other.position, graph);
                    DrawLine(texture, startPixel, endPixel, connectionLineColor, 1);
                });
            }
        });

        // 4. 최종 경로 그리기 (빨간 선)
        if (p != null && p.vectorPath != null)
        {
            for (int i = 0; i < p.vectorPath.Count - 1; i++)
            {
                Vector2Int startPixel = WorldToPixel(p.vectorPath[i], graph);
                Vector2Int endPixel = WorldToPixel(p.vectorPath[i+1], graph);
                DrawLine(texture, startPixel, endPixel, pathLineColor, pathLineThickness);
            }
        }

        texture.Apply();

        byte[] bytes = texture.EncodeToJPG();
        string path = System.IO.Path.Combine(Application.dataPath, "PathVisualization_Lines.jpg");
        File.WriteAllBytes(path, bytes);

        #if UNITY_EDITOR
        UnityEditor.AssetDatabase.Refresh();
        #endif

        Debug.Log($"<color=green>선 기반 경로 시각화 이미지를 저장했습니다: {path}</color>");
    }

    private Vector2Int WorldToPixel(Vector3 worldPos, GridGraph graph)
    {
        float pixelX = ((worldPos.x - graph.center.x + (graph.width * graph.nodeSize) / 2f) / (graph.width * graph.nodeSize)) * textureSize;
        float pixelZ = ((worldPos.z - graph.center.z + (graph.depth * graph.nodeSize) / 2f) / (graph.depth * graph.nodeSize)) * textureSize;
        return new Vector2Int(Mathf.Clamp((int)pixelX, 0, textureSize-1), Mathf.Clamp((int)pixelZ, 0, textureSize-1));
    }

    void DrawLine(Texture2D tex, Vector2Int p1, Vector2Int p2, Color col, int thickness)
    {
        int x = p1.x;
        int y = p1.y;
        int x2 = p2.x;
        int y2 = p2.y;

        int w = x2 - x;
        int h = y2 - y;
        int dx1 = 0, dy1 = 0, dx2 = 0, dy2 = 0;
        if (w < 0) dx1 = -1; else if (w > 0) dx1 = 1;
        if (h < 0) dy1 = -1; else if (h > 0) dy1 = 1;
        if (w < 0) dx2 = -1; else if (w > 0) dx2 = 1;
        int longest = Mathf.Abs(w);
        int shortest = Mathf.Abs(h);
        if (!(longest > shortest))
        {
            longest = Mathf.Abs(h);
            shortest = Mathf.Abs(w);
            if (h < 0) dy2 = -1; else if (h > 0) dy2 = 1;
            dx2 = 0;
        }
        int numerator = longest >> 1;
        for (int i = 0; i <= longest; i++)
        {
            for (int j = -thickness/2; j <= thickness/2; j++)
            {
                for (int k = -thickness/2; k <= thickness/2; k++)
                {
                    tex.SetPixel(x + j, y + k, col);
                }
            }
            numerator += shortest;
            if (!(numerator < longest))
            {
                numerator -= longest;
                x += dx1;
                y += dy1;
            }
            else
            {
                x += dx2;
                y += dy2;
            }
        }
    }
}