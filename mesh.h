#ifndef MESH_H
#define MESH_H

#include <vector>

#include <Eigen/StdVector>
#include <map>
#include <QSet>

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
    void subdivide();

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
        bool isNew = false;
    };

    struct Edge {
        Halfedge *halfedge;
    };

    struct Face {
        Halfedge *halfedge;
    };


    std::map<std::pair<int,int>, Halfedge*> m_edges;
    std::vector<Vertex*> m_verts;

    QSet<Face*> m_faces;
    QSet<Edge*> m_visited;

    Vertex *m_start;
    void flattenHalfedge(Face *face);
    void split(Edge *edge);
    void flip(Edge *edge);
    void subdivideRecursive(Halfedge *h);
};

#endif // MESH_H
