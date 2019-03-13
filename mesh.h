#ifndef MESH_H
#define MESH_H

#include <vector>

#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <map>
#include <iostream>
#include <QSet>
#include <Eigen/Dense>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3i)

class Mesh
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void initFromVectors(const std::vector<Eigen::Vector3f> &vertices,
         const std::vector<Eigen::Vector3i> &faces);
    void loadFromFile(const std::string &filePath);
    void saveToFile(const std::string &filePath);
    void convertToHalfedge();
    void convertFromHalfedge();
    void subdivide(int iterations);
    void simplify(int faces);
    void remesh(int iterations, float weight);

private:
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;


    struct Vertex;
    struct Edge;
    struct Face;

    struct Halfedge {
        Halfedge *twin;
        Halfedge *next;
        Vertex *vertex;
        Edge *edge;
        Face *face;
    };

    struct Vertex {
        Halfedge *halfedge;
        Eigen::Vector3f position;
        bool isNew;
        Eigen::Matrix4f quadric;
    };

    struct Edge {
        Halfedge *halfedge;
        bool isNew;
        float cost;
        Eigen::Vector3f v;
        Eigen::Matrix4f quadric;
    };

    struct Face {
        Halfedge *halfedge;
        Eigen::Matrix4f quadric;
    };

    QSet<Vertex*> m_newVerts;
    QSet<Vertex*> m_newVerts2;
    QSet<Edge*> m_edges;
    QSet<Face*> m_faces;
    QSet<Edge*> m_visited;
    QSet<Edge*> m_edges2;

    Vertex *m_start;
    void flattenHalfedge(Face *face);
    void split(Edge *edge, bool toMidpoint);
    void flip(Edge *edge);
    bool collapse(Edge *edge, bool toMidpoint);
    void splitRecursive(Halfedge *h);
    void getVertices(Vertex *vert);
    void falseEdges(Vertex *vert);
    void calcFaceQuadrics(Face *face);
    Eigen::Vector3f getP(Vertex *v1, Vertex *v2, Vertex *v3);
    int getDegree(Vertex *vert);
    void getEdges(Face *face);

    struct QComparator {
        bool operator() (Edge *e1, Edge *e2) {
            if (e1->cost == e2->cost) {
                return e1 < e2;
            } else {
                return e1->cost < e2->cost;
            }
        }
    };
};

#endif // MESH_H
