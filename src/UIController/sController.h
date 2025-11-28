#ifndef SCONTROLLER_H
#define SCONTROLLER_H
#include <opencv2/opencv.hpp>
///
/// \brief The sController class :  forme générale du correcteur
///                                 x[k+1]  = Ak x[k] + B1k r[k] + B2k y[k]
///                                 u[k]    = Ck x[k] + D1k r[k] + D2k y[k]
///

using namespace cv;

class sController{
public :
    sController(void);
    ~sController(void);
    // Méthode publique
    void init(void);
    void update(double *measures, double* references, double *control);
    void setControllerParameters(double *Param);

protected:
    unsigned int    nb_measures;
    unsigned int    nb_references;
    unsigned int    nb_control;
    unsigned int    nb_history;
    unsigned int    type_approximation;
    unsigned int    type;       // type de correcteur utilisé
                                //          0 : P
                                //          1 : PI
                                //          2 : PD
                                //          3 : PID
                                //          4 : PIDF
    Mat *A;//(30, 40, DataType<float>::type);
    double **pMeasures;         // Variable contenant les mesures courantes et anciennes
    double **pReferences;       // Variable contenant les consignes et anciennes
    double **pControl;          // Variable contenant les commandes et anciennes
    double *Parameters;               // Paramètres du correcteur
    double *Sat;                // Paramètres de saturation du correcteur

};
#endif // SCONTROLLER_H
