import numpy as np
from scipy import spatial



def sph2cart(r, az, el):
    """
    Converts spherical coordinates to cartesian coordinates.
    r    : range
    az   : azimuth (rad)
    el   : elevation (rad)
    returns   a (3,) numpy vector [x, y, z]
    """

    x = r * np.cos(el) * np.cos(az)
    y = r * np.cos(el) * np.sin(az)
    z = r * np.sin(el)
    return np.array([x, y, z])

def R_z(rad):
    """
    Generates rotation matrix around the Z-axis
    """
    return np.array(
        [
            [np.cos(rad), -np.sin(rad), 0],
            [np.sin(rad),  np.cos(rad), 0],
            [          0,            0, 1]
        ]
    )

def R_y(rad): 
    return np.array(
        [
            [ np.cos(rad), 0, np.sin(rad)],
            [           0, 1,           0],
            [-np.sin(rad), 0, np.cos(rad)]
        ]
    )
def R_x(rad):
    return np.array(
        [
            [1,           0,            0],
            [0, np.cos(rad), -np.sin(rad)],
            [0, np.sin(rad),  np.cos(rad)]
        ]
    )


class Transformer():

    def __init__(self, translations=[0,0,0], rotations=[0,0,0], degrees=True):
        """
        Creates a transformer to translate and rotate vectors in 3D.

        translations: Vector containing translations that will be performed on inputted vect
ors. Format [x, y, z]
        rotations: Vector containing rotations that will be applied on to the inputted vecto
rs. Format [x, y, z]
        degrees: Whether the rotations should be interpreted as degrees or radians. 


        """
        
        self.translations = translations
        self.rotations = rotations
        self._rotator = spatial.transform.Rotation.from_euler("xyz", [rotations[0], rotations[1], rotations[2]], degrees=degrees)
        self.degrees = degrees

    def apply(self, v: np.ndarray) -> np.ndarray:
        """
        Apply transformation to vector or sequence of vectors in 3D.
        """

        rotated_vec = self._rotator.apply(v)
        return rotated_vec + self.translations



def main():

    transformer_global = Transformer()
    transformer_1 = Transformer(rotations=[90, 180, 0], translations=[-10, -10, -10])

    v1 = [1, 0, 0]
    v2 = [1, 0, 0]

    t1 = transformer_global.apply(v1)
    t2 = transformer_1.apply(v2)

    print(v1, "->", t1)
    print(v2, "->", t2)

if __name__ == "__main__":
    main()
