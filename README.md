# face-fusing
Image morphing that fuses two face images. Impletemented in C++ with CImg.

## Example
![](https://github.com/dalegebit/face-fusing/blob/master/example/1to2.gif)

---
All source codes are included in `src`.
Two test images are included in `res/pics`  
The control points are included in `res/control_pts`.
All result images (and gif) are included in `example`.



Open `FaceFusing.sln` in Visual Studio and rebuild the project, test program `FaceFusing.exe` will appear in `Release` folder.

---

## Details of the Algorithm
For now, control points are pre-selected manually using matlab image processing tool and stored in a txt.
The format is like:
```
100 100

102 100

...
```
Dulaunay Triangulation is implemented with Bowyer-Watson algorithm, which is an incremented algorithm.
Refs: https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm

For each fusing procedure, the algorithm needs to know the correspongding positions in source image and in targe image of a pixel in the fusing image, which means the two translation functions should be determined, so that its rgb value can be an interpolation of the rgb value of its corresponding positions in the two images. In this algorithm, we do this:


1. Interpolate the control points of the fusing image using the source image's and target image's cps. `T(M) = O(M)`
2. Obtain delaunay triangles through delaunay triangulation. `T(M) = O(MlogM)`, although special degenerate cases exist where this goes up to `T(M) = O(M^2)`
3. Paint each triangle in a colorspace the same size as the fusing image with the color representing its index in the triangle list so that we can fast obtain which triangle a point is included in based on its color in its corresponding position in the colorspace. `T(N) = O(N)`
4. For each triangle, find its corresponding triangle in source's triangles and target's triangles with plain linear search. `T(M) = O(M^2)`
5. For each triangle, compute the affine translation matrices from fusing image to source image and to target image.   `T(M) = O(M)`
6. For each pixel in fusing image, calculate the rgb value of its corresponding positions in source image and target image with the translation matrix corresponding to the triangle the pixel is included in, and interpolate these two rgb value.

`(T(n) : time complexity, N : number of pixels, M : number of control points)`

The time complexity of the whole algorithm for generating a single funsing image is `O(N + M^2)`. If `M <= sqrt(N)`, which usually is the reality, the complexity becomes `O(N)`, which is very promising.

