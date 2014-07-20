#ifndef MPU_H
#define MPU_H

unsigned short inv_row_2_scale(const signed char *row);
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);

#endif
