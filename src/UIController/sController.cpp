#include "sController.h"
#include <stddef.h>

sController::sController(void)
{

}

sController::~sController()
{

}



///
/// \fn     sController::setControllerParameters
/// \brief  Méthode permettant de renseigner les paramètres/coefficients du correcteur
/// \param  Param
///
void sController::setControllerParameters(double *Param)
{
    if ( Param == NULL)
        return;
    Parameters = Param;

}
