#ifndef __QMI8658C_H
#define __QMI8658C_H

#include <stdint.h>

/* Pins confirmed from 07_ADS1292_260115_ZSM(1).pdf. */
#define QMI_TWI_INST 1
#define QMI_SDA      22  /* P0.22 */
#define QMI_SCL      32  /* P1.00 */
#define QMI_INT1     20  /* P0.20 */
#define QMI_INT2     34  /* P1.02 */

void qmi8658c_read(void * pvParameter);

#endif
