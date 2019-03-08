#include "mesh.h"
#include <iostream>
#include <fstream>
#include <QFileInfo>
#include <QString>

#define TINYOBJLOADER_IMPLEMENTATION
#include "util/tiny_obj_loader.h"

using namespace Eigen;
using namespace std;

void Mesh::initFromVectors(const std::vector<Vector3f> &vertices,
           const std::vector<Vector3i> &faces)
{
    _vertices = vertices;
    _faces = faces;
}

void Mesh::loadFromFile(const std::string &filePath)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    QFileInfo info(QString(filePath.c_str()));
    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
                                info.absoluteFilePath().toStdString().c_str(), (info.absolutePath().toStdString() + "/").c_str(), true);
    if(!err.empty()) {
        std::cerr << err << std::endl;
    }

    if(!ret) {
        std::cerr << "Failed to load/parse .obj file" << std::endl;
        return;
    }

    for(size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            unsigned int fv = shapes[s].mesh.num_face_vertices[f];

            Vector3i face;
            for(size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

		face[v] = idx.vertex_index;

            }
            _faces.push_back(face);

            index_offset += fv;
        }
    }
    for(size_t i = 0; i < attrib.vertices.size(); i += 3) {
	_vertices.emplace_back(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]);
    }
    std::cout << "Loaded " << _faces.size() << " faces and " << _vertices.size() << " vertices" << std::endl;
}


void Mesh::saveToFile(const std::string &filePath)
{
    std::ofstream outfile;
    outfile.open(filePath);

    // Write vertices
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        const Vector3f &v = _vertices[i];
        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
    }

    // Write faces
    for (size_t i = 0; i < _faces.size(); i++)
    {
        const Vector3i &f = _faces[i];
        outfile << "f " << (f[0]+1) << " " << (f[1]+1) << " " << (f[2]+1) << std::endl;
    }

    outfile.close();
}

void Mesh::convertToHalfedge() {

    //initialize vertices
    int numVerts = _vertices.size();
    m_verts.resize(numVerts);
    for (int i = 0; i < numVerts; i++) {
        m_verts[i] = new Vertex();
        m_verts[i]->position = _vertices[i];
    }

    m_start = m_verts[0];

    // initialize half-edges
    for (auto face : _faces) {
        for (int i = 0; i < 3; i++) {
            int j = i + 1;
            if (j >= 3) j = 0;
            pair<int,int> edge(face[i], face[j]);
            auto he = new Halfedge();
            m_edges[edge] = he;
            he->vertex = m_verts[face[i]];

            // not really an issue, but self-note: vertex half-edge will be reassigned for every single edge coming from vertex
            he->vertex->halfedge = he;
        }

        for (int i = 0; i < 3; i++) {
            int j = i + 1;
            if (j >= 3) j = 0;
            int k = j + 1;
            if (k >= 3) k = 0;
            m_edges[pair<int,int>(face[i], face[j])]->next = m_edges[pair<int,int>(face[j], face[k])];
            if (m_edges.find(pair<int,int>(face[j], face[i])) != m_edges.end()) {
                m_edges[pair<int,int>(face[i], face[j])]->twin = m_edges[pair<int,int>(face[j], face[i])];
                m_edges[pair<int,int>(face[j], face[i])]->twin = m_edges[pair<int,int>(face[i], face[j])];
            }
        }
    }

    //initialize everything that depends on half edges

    for (auto face : _faces) {
        auto faceObj = new Face();
        faceObj->halfedge = m_edges[pair<int,int>(face[0], face[1])];
        for (int i = 0; i < 3; i++) {
            int j = i + 1;
            if (j >= 3) j = 0;
            pair<int,int> edge(face[i], face[j]);
            m_edges[edge]->face = faceObj;

            if (j > i) {
                auto edgeObj = new Edge();
                edgeObj->halfedge = m_edges[pair<int,int>(face[i],face[j])];
                m_edges[pair<int,int>(face[i], face[j])]->edge = edgeObj;
                m_edges[pair<int,int>(face[j], face[i])]->edge = edgeObj;
            }
        }
    }

}

void Mesh::convertFromHalfedge() {
    // I'll be taking advantage of the fact that c++ sets are ordered
    QSet<Vertex *> vertexPtrs;
    std::vector<Vector3f> newVertices;
    std::vector<Vector3i> newFaces;
    flattenHalfedge(m_start->halfedge->face);
    for (auto face : m_faces) {
        auto h = face->halfedge;
        do {
            vertexPtrs.insert(h->vertex);
            h = h->next;
        } while (h != face->halfedge);
    }

    newFaces.reserve(m_faces.size());
    std::vector<Vertex *> vecVert(vertexPtrs.begin(), vertexPtrs.end());
    for (auto face : m_faces) {
        std::vector<int> vIndex;
        vIndex.reserve(3);
        auto h = face->halfedge;
        do {
            auto it = std::find(vecVert.begin(), vecVert.end(), h->vertex);
            vIndex.push_back(std::distance(vecVert.begin(), it));
            h = h->next;
        } while (h != face->halfedge);
        newFaces.push_back(Vector3i(vIndex[0], vIndex[1], vIndex[2]));
    }

    newVertices.reserve(vertexPtrs.size());
    for (auto vert : vertexPtrs) {
        newVertices.push_back(vert->position);
    }

    _faces = newFaces;
    _vertices = newVertices;
}
void Mesh::flattenHalfedge(Face *face) {
    if (m_faces.contains(face)) return;
    m_faces.insert(face);
    auto h = face->halfedge;
    do {
        flattenHalfedge(h->twin->face);
        h = h->next;
    } while (h != face->halfedge);
}

void Mesh::subdivide() {
    auto edge = m_start->halfedge->edge;
    auto nextEdge = m_start->halfedge->next->edge;
    auto prevEdge = m_start->halfedge->next->next->edge;
    flip(edge);
    //    subdivideRecursive(edge);
//    subdivideRecursive(nextEdge);
//    subdivideRecursive(prevEdge);
}

int m_count = 0;
void Mesh::subdivideRecursive(Edge *edge) {
    if(m_visited.contains(edge)) {

        return;
    }
    m_count++;
    auto t1 = edge->halfedge;
    auto t2 = t1->twin;
    auto t3 = t2->next;
    auto t4 = t3->edge;
    auto nextEdge = edge->halfedge->twin->next->edge;
    auto prevEdge = edge->halfedge->twin->next->next->edge;
    std::cout<<m_count<<std::endl;
    split(edge);
    subdivideRecursive(nextEdge);
    subdivideRecursive(prevEdge);

}

void Mesh::nextEdges(Edge *edge) {
    auto t = edge->halfedge->twin->next;
    subdivideRecursive(t->edge);
    subdivideRecursive(t->next->edge);
}

void Mesh::split(Edge *edge) {
    auto h = edge->halfedge;
    auto v0 = h->next->next->vertex;
    auto v1 = h->twin->next->next->vertex;

    //This is about to get very confusing, so I'm going to organize everything by cardinal direction
    auto newHN = new Halfedge();
    auto newTN = new Halfedge();
    auto newHS = new Halfedge();
    auto newTS = new Halfedge();
    auto newHW = new Halfedge();
    auto newTW = new Halfedge();
    auto newHE = new Halfedge();
    auto newTE = new Halfedge();

    auto newEN = new Edge();
    auto newES = new Edge();
    auto newEW = new Edge();
    auto newEE = new Edge();
    newEN->halfedge = newHN;
    newES->halfedge = newHS;
    newEW->halfedge = newHW;
    newEE->halfedge = newHE;
    m_visited.insert(newEW);
    m_visited.insert(newEE);
    m_visited.insert(newEN);
    m_visited.insert(newES);

    auto newHNF = new Face();
    auto newTNF = new Face();
    auto newHSF = new Face();
    auto newTSF = new Face();
    newHNF->halfedge = newHN;
    newTNF->halfedge = newTN;
    newHSF->halfedge = newHS;
    newTSF->halfedge = newTS;

    auto newV = new Vertex();
    newV->isNew = true;
    newV->halfedge = newHN;
    //newV->position = Vector3f(0, 0, 0);

    newHN->edge = newEN;
    newHN->face = newHNF;
    newHN->next = h->next->next;
    newHN->twin = newTN;
    newHN->vertex = newV;

    newTN->edge = newEN;
    newTN->face = newTNF;
    newTN->twin = newHN;
    newTN->vertex = v0;
    newTN->next = newHE;

    newHS->edge = newES;
    newHS->face = newHSF;
    newHS->twin = newTS;
    newHS->vertex = v1;
    newHS->next = newTW;

    newTS->edge = newES;
    newTS->face = newTSF;
    newTS->twin = newHS;
    newTS->vertex = newV;
    newTS->next = h->twin->next->next;

    newHW->edge = newEW;
    newHW->face = newHNF;
    newHW->twin = newTW;
    newHW->vertex = h->vertex;
    newHW->next = newHN;

    newTW->edge = newEW;
    newTW->face = newHSF;
    newTW->twin = newHW;
    newTW->vertex = newV;
    newTW->next = h->twin->next;

    newHE->edge = newEE;
    newHE->face = newTNF;
    newHE->twin = newTE;
    newHE->vertex = newV;
    newHE->next = h->next;

    newTE->edge = newEE;
    newTE->face = newTSF;
    newTE->twin = newHE;
    newTE->vertex = h->twin->vertex;
    newTE->next = newTS;

//    if (h->twin->vertex == v1) h->twin->vertex->halfedge = h->next;
//    h->twin->vertex = newV;
//    h->twin->face = newHSF;
//    h->face = newHNF;
//    h->next = newHN;

    if (h->vertex->halfedge == h) h->vertex->halfedge = newHW;
    if (h->twin->vertex->halfedge == h->twin) h->twin->vertex->halfedge = newTE;
    delete h->face;
    delete h->twin->face;
    delete h->edge;
    delete h->twin;
    delete h;

}

void Mesh::flip(Edge *edge) {
    auto h = edge->halfedge;
    auto t = h->twin;
    auto v0 = h->vertex;
    auto v1 = t->vertex;

    //check if flip is valid
    int degreeV0 = 0;
    int degreeV1 = 0;
    auto vh = v0->halfedge;
    auto vt = v1->halfedge;
    do {
        degreeV0++;
        vh = vh->twin->next;
    } while (vh != v0->halfedge);

    do {
        degreeV1++;
        vt = vt->twin->next;
    } while (vt != v1->halfedge);
    std::cout<<degreeV0<<","<<degreeV1<<std::endl;
    if (degreeV0 == 3 || degreeV1 == 3) return;

    auto prevH = h->next->next;
    auto prevT = t->next->next;

    // Reassigning vertex half edge just in case
    if (v0->halfedge == h) v0->halfedge = prevH->twin;
    if (v1->halfedge == t) v1->halfedge = prevT->twin;

    auto newV0 = prevH->vertex;
    auto newV1 = prevT->vertex;

    auto newH = new Halfedge();
    auto newT = new Halfedge();

    auto faceH = new Face();
    auto faceT = new Face();
    auto newEdge = new Edge();
    newEdge->halfedge = newH;
    faceH->halfedge = newH;
    faceT->halfedge = newT;

    newH->edge = newEdge;
    newH->twin = newT;
    newH->face = faceH;
    newH->vertex = newV0;
    newH->next = prevH;

    newT->edge = newEdge;
    newT->face = faceT;
    newT->twin = newH;
    newT->vertex = newV1;
    newT->next = prevT;

    delete h->face;
    delete t->face;
    delete h->edge;
    delete t;
    delete h;
}
