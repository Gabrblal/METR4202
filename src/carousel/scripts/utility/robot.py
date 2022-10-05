from numpy import asarray, hstack, vstack, cross

class Robot:

    def __init__(self, w, v, M):
        self._screws = asarray(
            vstack(
                [hstack([wi, -cross(wi, vi)]) for wi, vi in zip(w, v)]
            )
        )
        self._M = asarray(M)

    def __str__(self):
        return '\n'.join([
            f'Robot(',
            f'    screws = [',
          *[f'        {tuple(row)}' for row in self._screws],
            f'    ],',
            f'    M = [',
          *[f'        {tuple(row)}' for row in self._M],
            f'    ]',
            f')'
        ])

    @property
    def screws(self):
        return self._screws

    @property
    def M(self):
        return self._M

carousel = Robot(
    [
        (0, 0, 1),
        (1, 0, 0),
        (1, 0, 0),
        (1, 0, 0)
    ], [
        (0, 0, 0),
        (0, 0, 54),
        (0, 0, 171),
        (0, 0, 265)
    ], [
        [1, 0, 0, 0],
        [0, 0, -1, 12],
        [0, 1, 0, 303],
        [0, 0, 0, 1],
    ]
)

if __name__ == '__main__':
    print(carousel)
