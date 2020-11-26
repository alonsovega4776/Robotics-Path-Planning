"""
Alonso Vega
November 25, 2020
Camera by Jacques Kvam (https://github.com/jwkvam)
modified by Alonso Vega
Easy matplotlib animation.
"""
from typing import Dict, List
from collections import defaultdict

from matplotlib.figure import Figure
from matplotlib.artist import Artist
from matplotlib.animation import ArtistAnimation

__version__ = '0.2.0'


class Camera:
    """Make animations easier."""

    def __init__(self, figure: Figure) -> None:
        """Create camera from matplotlib figure."""
        self._figure = figure
        # need to keep track off artists for each axis
        self._offsets: Dict[str, Dict[int, int]] = {
            k: defaultdict(int) for k in [
                'collections', 'patches', 'lines', 'texts', 'artists', 'images'
            ]
        }
        self._photos: List[List[Artist]] = []
        self._static_photos: List[Artist] = []

    def snap(self, dynamic_art) -> List[Artist]:
        """Capture current state of the figure."""
        new_frame_artists: List[Artist] = []
        for i, axis in enumerate(self._figure.axes):
            if axis.legend_ is not None:
                axis.add_artist(axis.legend_)
            for name in self._offsets:
                new_artists = getattr(axis, name)[self._offsets[name][i]:]
                new_frame_artists += new_artists
                self._offsets[name][i] += len(new_artists)

        end = len(self._photos)
        if end > 0:
            """"
            self._photos.append([])
            for old_art in self._photos[end-1]:
                self._photos[end].append(old_art)
            for new_art in new_frame_artists:
                self._photos[end].append(new_art)
            # """
            self._photos.append([])
            for static_art in self._static_photos:
                self._photos[end].append(static_art)
            for new_art in new_frame_artists:
                self._photos[end].append(new_art)
        else:
            self._photos.append(new_frame_artists)
            for first_art in new_frame_artists:
                if first_art not in dynamic_art:
                    self._static_photos.append(first_art)
        return new_frame_artists

    def animate(self, *args, **kwargs) -> ArtistAnimation:
        """Animate the snapshots taken.

        Uses matplotlib.animation.ArtistAnimation

        Returns
        -------
        ArtistAnimation

        """
        return ArtistAnimation(self._figure, self._photos, *args, **kwargs)
