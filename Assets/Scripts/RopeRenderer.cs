using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Profiling;

[RequireComponent(typeof(MeshRenderer))]
[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(VerletRope))]
public class RopeRenderer : MonoBehaviour
{
    [Min(3)] [SerializeField] private int m_RopeSegmentSides;

    private MeshFilter m_MeshFilter;
    private MeshRenderer m_MeshRenderer;
    private Mesh m_RopeMesh;
    private VerletRope m_Rope;
    private Vector3[] m_Vertices;
    private int[] m_Triangles;

    private float m_Angle;
    private int m_NodeCount;
    private bool m_IsInitialized;

    private ComputeVerticesJob _computeVerticesJob;
    private ComputeTrianglesJob _computeTrianglesJob;

    private void Awake()
    {
        m_MeshFilter = GetComponent<MeshFilter>();
        m_MeshRenderer = GetComponent<MeshRenderer>();

        m_RopeMesh = new Mesh();
        m_Angle = ((m_RopeSegmentSides - 2) * 180) / m_RopeSegmentSides;
        m_IsInitialized = false;

        _computeVerticesJob = new ComputeVerticesJob();
        _computeTrianglesJob = new ComputeTrianglesJob();
    }

    private void Start()
    {
        m_Rope = GetComponent<VerletRope>();
        m_Vertices = new Vector3[m_Rope.GetNodeCount() * m_RopeSegmentSides];
        m_Triangles = new int[m_RopeSegmentSides * (m_Rope.GetNodeCount() - 1) * 6];
    }

    public void RenderRope(VerletNode[] nodes, float radius)
    {
        if (m_Vertices is null || m_Triangles is null) return;
        
        ComputeVertices(nodes, radius);
        
        if(!m_IsInitialized)
        {
            ComputeTriangles();
            m_IsInitialized = true;
        }
        
        SetupMeshFilter();
    }

    private void ComputeVertices(VerletNode[] nodes, float radius)
    {
        var angle = (360f / m_RopeSegmentSides) * Mathf.Deg2Rad;

        _computeVerticesJob.Angle = angle;
        _computeVerticesJob.Radius = radius;
        _computeVerticesJob.RopeSegmentSides = m_RopeSegmentSides;
        
        var nodesNative = new NativeArray<VerletNode>(nodes, Allocator.TempJob);
        _computeVerticesJob.Nodes = nodesNative;
        
        var vertices = new NativeArray<Vector3>(m_Vertices, Allocator.TempJob);
        _computeVerticesJob.Vertices = vertices;
        
        _computeVerticesJob.Schedule(m_Vertices.Length, 6).Complete();
        
        m_Vertices = vertices.ToArray();

        nodesNative.Dispose();
        vertices.Dispose();
    }
    
    [BurstCompile]
    private struct ComputeVerticesJob: IJobParallelFor
    {
        [ReadOnly] public float Radius;
        [ReadOnly] public float Angle;
        [ReadOnly] public int RopeSegmentSides;
        [ReadOnly] public NativeArray<VerletNode> Nodes;
        [WriteOnly] public NativeArray<Vector3> Vertices;
        
        public void Execute(int i)
        {
            var nodeindex = i / RopeSegmentSides;
            var sign = nodeindex == Nodes.Length - 1 ? -1 : 1;
            
            var currNodePosition = Nodes[nodeindex].Position;
            var normalOfPlane =
                (sign * Nodes[nodeindex].Position + -sign * Nodes[nodeindex + (nodeindex == Nodes.Length - 1 ? -1 : 1)].Position)
                .normalized;

            var u = Vector3.Cross(normalOfPlane, Vector3.forward).normalized;
            var v = Vector3.Cross(u, normalOfPlane).normalized;

            Vertices[i] = currNodePosition + Radius * Mathf.Cos(Angle * (i % RopeSegmentSides)) * u +
                            Radius * Mathf.Sin(Angle * (i % RopeSegmentSides)) * v;
        }
    }

    private void ComputeTriangles()
    {
        _computeTrianglesJob.tn = 0;
        _computeTrianglesJob.RopeSegmentSides = m_RopeSegmentSides;
        var triangles = new NativeArray<int>(m_Triangles, Allocator.TempJob);
        _computeTrianglesJob.Triangles = triangles;
        
        _computeTrianglesJob.Schedule(m_Vertices.Length - m_RopeSegmentSides, 6).Complete();

        m_Triangles = triangles.ToArray();
        triangles.Dispose();
    }
    
    [BurstCompile]
    private struct ComputeTrianglesJob : IJobParallelFor
    {
        public int tn;
        [ReadOnly] public int RopeSegmentSides;
        [NativeDisableParallelForRestriction] public NativeArray<int> Triangles;
        
        public void Execute(int i)
        {
            var nexti = (i + 1) % RopeSegmentSides == 0 ? i - RopeSegmentSides + 1 : i + 1;

            Triangles[tn] = i;
            Triangles[tn + 1] = nexti + RopeSegmentSides;
            Triangles[tn + 2] = i + RopeSegmentSides;

            Triangles[tn + 3] = i;
            Triangles[tn + 4] = nexti;
            Triangles[tn + 5] = nexti + RopeSegmentSides;

            tn += 6;
        }
    }

    private void SetupMeshFilter()
    {
        for (int i = 0; i < m_Vertices.Length; i++)
        {
            m_Vertices[i] -= transform.position;
        }
        
        m_RopeMesh.Clear();
        m_RopeMesh.vertices = m_Vertices;
        m_RopeMesh.triangles = m_Triangles;

        m_MeshFilter.mesh = m_RopeMesh;
        m_MeshFilter.mesh.RecalculateBounds();
        m_MeshFilter.mesh.RecalculateNormals();
    }
}