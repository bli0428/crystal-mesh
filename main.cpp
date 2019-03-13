#include <QCoreApplication>
#include <QCommandLineParser>

#include <iostream>
#include <chrono>

#include "mesh.h"

using namespace std;
using namespace std::chrono;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    ////////////////////////////////////////////////////////////////////////////////
    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("infile", "Input .obj file path");
    parser.addPositionalArgument("outfile", "Output .obj file path");
    parser.addPositionalArgument("method", "subdivide/simplify/remesh/denoise");

    //Subdivide: number of iterations
    //Simplify: number of faces to remove
    //Remesh: number of iterations
    //Denoise: number of iterations
    parser.addPositionalArgument("args1", "respective argument for the method");

    //Remesh: Tangential smoothing weight
    //Denoise: Smoothing parameter 1 (\Sigma_c)
    parser.addPositionalArgument("args2", "respective argument2 for the method");

    //Denoise: Smoothing parameter 2 (\Sigma_s)
    parser.addPositionalArgument("args3", "respective argument3 for the method");

    //Denoise: Kernel size (\rho)
    parser.addPositionalArgument("args4", "respective argument4 for the method");

    parser.process(a);

    const QStringList args = parser.positionalArguments();
    if(args.size() < 3) {
        cerr << "Error: Wrong number of arguments" << endl;
        a.exit(1);
        return 1;
    }
    QString infile = args[0];
    QString outfile = args[1];
    QString method = args[2];

    ////////////////////////////////////////////////////////////////////////////////

    Mesh m;
    m.loadFromFile(infile.toStdString());

    auto t0 = high_resolution_clock::now();
    // TODO
    // Convert the mesh into your own data structure
    m.convertToHalfedge();

    // TODO
    // Implement the operations
    if (method == "subdivide"){
        if (args.size() > 3) {
            m.subdivide(args[3].toInt());
        } else {
            m.subdivide(1);
        }
    } else if (method == "simplify"){
        //TODO
            if (args.size() > 3) {
                m.simplify(args[3].toInt());
            } else {
                m.simplify(1);
            }
    } else if (method == "remesh"){
        //TODO
        if (args.size() > 5) {
            m.remesh(args[3].toInt(), args[4].toFloat());
        } else if (args.size() == 4) {
            m.remesh(args[3].toInt(), 0.5f);
        } else {
            m.remesh(1, 0.5f);
        }

    } else if (method == "denoise") {
        //TODO
    } else {
        cerr << "Error: Unknown method name" << endl;
    }

    auto t1 = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(t1-t0).count();

    cout << "Execution takes: " << duration << " milliseconds." <<endl;
    // TODO
    // Convert your datastructure back to the basic format
    m.convertFromHalfedge();

    ////////////////////////////////////////////////////////////////////////////////
    m.saveToFile(outfile.toStdString());

    a.exit();
}
