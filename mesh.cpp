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
        m_verts[i]->isNew = false;
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
                auto edgeObj = new Edge();
                edgeObj->isNew = false;
                auto hij = m_edges[pair<int,int>(face[i], face[j])];
                auto hji = m_edges[pair<int,int>(face[j], face[i])];
                edgeObj->halfedge = hij;
                hij->twin = hji;
                hji->twin = hij;
                hij->edge = edgeObj;
                hji->edge = edgeObj;
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

void Mesh::subdivide(int iterations) {
    for (int i = 0; i < iterations; i++) {

    auto h = m_start->halfedge;
    auto nextEdge = m_start->halfedge->next;
    auto prevEdge = m_start->halfedge->next->next;;
    subdivideRecursive(h);
    subdivideRecursive(nextEdge);
    subdivideRecursive(prevEdge);

    for (auto edge : m_visited) {
        if (edge->isNew && edge->halfedge->vertex->isNew != edge->halfedge->twin->vertex->isNew) {
            flip(edge);
        }
        edge->isNew = false;
    }


    const float pi = 3.141592653589793f;
    for (auto vert : m_verts) {
        // Get old vertices
        int n = 0;
        auto vh = vert->halfedge;
        std::vector<Vertex*> reweight;
        do {
            n++;
            auto reVert = vh->next->twin->next->twin->next->twin->vertex;
            //auto reVert = vh->twin->vertex;
            reweight.push_back(reVert);
            vh = vh->twin->next;
        } while (vh != vert->halfedge);
        float u = (1.f/n) * (5.f/8 - std::pow(3.f/8 + std::cos(2 * pi/n)/4.f, 2));
        Vector3f finalWeight = Vector3f(0,0,0);
        for (auto v : reweight) {
            finalWeight += v->position * u;
        }
        vert->position = vert->position * (1 - n*u) + finalWeight;
    }
    }
}



void Mesh::subdivideRecursive(Halfedge *h) {
    if(m_visited.contains(h->edge)) {
        return;
    }
    auto nextEdge = h->next->twin;
    auto prevEdge = h->next->next->twin;

    split(h->edge);
    subdivideRecursive(nextEdge);
    subdivideRecursive(prevEdge);
}


void Mesh::split(Edge *edge) {

    auto h = edge->halfedge;
    auto prevH = h->next->next;
    auto prevT = h->twin->next->next;
    auto v0 = prevH->vertex;
    auto v1 = prevT->vertex;
    auto hVert = h->vertex;
    auto tVert = h->twin->vertex;
    auto hNext = h->next;
    auto tNext = h->twin->next;

    //This is about to get very confusing, so I'm going to organize everything by cardinal direction
    auto newHN = new Halfedge();
    auto newTN = new Halfedge();
    auto newHS = new Halfedge();
    auto newTS = new Halfedge();
    auto newHE = new Halfedge();
    auto newTE = new Halfedge();

    auto newEN = new Edge();
    auto newES = new Edge();
    auto newEE = new Edge();
    newEN->halfedge = newHN;
    newES->halfedge = newHS;
    newEE->halfedge = newHE;
    newEN->isNew = true;
    newES->isNew = true;
    newEE->isNew = false;
    m_visited.insert(h->edge);
    m_visited.insert(newEE);
    m_visited.insert(newEN);
    m_visited.insert(newES);

    auto newTNF = new Face();
    auto newTSF = new Face();
    newTNF->halfedge = newTN;
    newTSF->halfedge = newTS;
    h->face->halfedge = h;
    h->twin->face->halfedge = h->twin;
    hNext->face = newTNF;
    prevT->face = newTSF;

    auto newV = new Vertex();
    newV->isNew = true;
    newV->halfedge = newHN;;

    newHN->edge = newEN;
    newHN->face = h->face;
    newHN->next = prevH;
    newHN->twin = newTN;
    newHN->vertex = newV;

    newTN->edge = newEN;
    newTN->face = newTNF;
    newTN->twin = newHN;
    newTN->vertex = v0;
    newTN->next = newHE;

    newHS->edge = newES;
    newHS->face = h->twin->face;
    newHS->twin = newTS;
    newHS->vertex = v1;
    newHS->next = h->twin;

    newTS->edge = newES;
    newTS->face = newTSF;
    newTS->twin = newHS;
    newTS->vertex = newV;
    newTS->next = prevT;


    h->next = newHN;

    h->twin->vertex = newV;

    newHE->edge = newEE;
    newHE->face = newTNF;
    newHE->twin = newTE;
    newHE->vertex = newV;
    newHE->next = hNext;

    newTE->edge = newEE;
    newTE->face = newTSF;
    newTE->twin = newHE;
    newTE->vertex = tVert;
    newTE->next = newTS;

    hNext->next = newTN;
    tNext->next = newHS;
    prevT->next = newTE;

    if (tVert->halfedge == h->twin) tVert->halfedge = newTE;

    newV->position = hVert->position * (3.f/8) + tVert->position * (3.f/8)
            + v0->position * (1.f/8) + v1->position * (1.f/8);
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
    if (degreeV0 == 3 || degreeV1 == 3) return;

    auto prevH = h->next->next;
    auto prevT = t->next->next;

    // Reassigning vertex half edge just in case
    if (v0->halfedge == h) v0->halfedge = prevH->twin;
    if (v1->halfedge == t) v1->halfedge = prevT->twin;

    auto newV0 = prevT->vertex;
    auto newV1 = prevH->vertex;

    h->vertex = newV0;
    t->vertex = newV1;
    h->vertex->halfedge = h;
    t->vertex->halfedge = t;

    h->face->halfedge = h;
    t->face->halfedge = t;

    auto hNext = h->next;
    auto tNext = t->next;
    h->next = prevH;
    t->next = prevT;

    h->next->face = h->face;
    t->next->face = t->face;

    h->next->next = tNext;
    t->next->next = hNext;

    h->next->next->face = h->face;
    h->next->next->face = h->face;

    h->next->next->next = h;
    t->next->next->next = t;
}

//void Mesh::getVertices(Vertex *vert) {
//    if (m_newVerts.contains(vert)) return;
//    m_newVerts.insert(vert);
//    auto h = vert->halfedge;
//    do {
//        getVertices(h->twin->vertex);
//        h = h->next->twin;
//    } while (vert != h->vertex);
//}

