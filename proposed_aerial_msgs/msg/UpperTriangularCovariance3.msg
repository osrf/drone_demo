# targeted for geometry_msgs

# Row-major representation of 6x6 velocity cross-covariance matrix in an upper right triangular
# Order: x, y, z

# If covariance invalid/unknown, first cell is NaN

# Covariance matrix index constants
uint8 MATRIX_X_VARIANCE=0
uint8 MATRIX_XY_VARIANCE=1
uint8 MATRIX_XZ_VARIANCE=3
uint8 MATRIX_Y_VARIANCE=3
uint8 MATRIX_YZ_VARIANCE=4
uint8 MATRIX_Z_VARIANCE=5

# Constants to index to cells to check for invalid/unknown data
uint8 MATRIX_VALID_CELL=0

# Raw Storage
float32[6] covariance