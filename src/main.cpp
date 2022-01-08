#include <iostream>
#include <utility>
#include <vector>
#include "linear/Vec.h"
#include "linear/Mat.h"
#include "tgaimage/tgaimage.h"

using namespace std;

#define min(a, b) ((a)<(b)?(a):(b))
#define max(a, b) ((a)>(b)?(a):(b))

static TGAColor interpolate(const TGAColor &c1, const TGAColor &c2, double t)
{
    t = max(min(1, t), 0);
    return {static_cast<unsigned char>(c1.r + (c2.r - c1.r) * t),
            static_cast<unsigned char>(c1.g + (c2.g - c1.g) * t),
            static_cast<unsigned char>(c1.b + (c2.b - c1.b) * t),
            static_cast<unsigned char>(c1.a + (c2.a - c1.a) * t)};
}

static inline TGAColor makeColor(const Vec4 &v)
{
    return {static_cast<unsigned char>(v[0]), static_cast<unsigned char>(v[1]), static_cast<unsigned char>(v[2]),
            static_cast<unsigned char>(v[3])};
}

static inline Vec4 convertColorToVec4(const TGAColor &c)
{
    return {static_cast<double>(c.r), static_cast<double>(c.g), static_cast<double>(c.b), static_cast<double>(c.a)};
}


struct Point : public Vec3
{
    TGAColor color{};

    Point() = default;

    Point(const Vec3 &v, const TGAColor &color) : Vec3(v), color(color) {}

    [[nodiscard]] inline Vec3 toVec3() const
    {
        return Vec3(arr[0], arr[1], arr[2]);
    }

};

struct Line
{
    Point p1;
    Point p2;

    Line() = default;

    Line(Point p1, Point p2) : p1(std::move(p1)), p2(std::move(p2)) {}
};

struct Triangle
{
    Point p1;
    Point p2;
    Point p3;

    Triangle() = default;

    Triangle(Point p1, Point p2, Point p3) : p1(std::move(p1)), p2(std::move(p2)), p3(std::move(p3)) {}
};


class Image : public TGAImage
{
private:

    struct Pixel
    {
        int x{}, y{};
        TGAColor c;

        Pixel() = default;

        Pixel(int x, int y, const TGAColor &c) : x(x), y(y), c(c) {};


    };

    Mat4 mtRes;

    [[nodiscard]] inline iVec3 transform(const Point &p) const
    {
        auto v = mtRes * Vec4(p.toVec3(), 1);
        v.multiple(1 / v[3]);
        return iVec3(round(v));
    }

    static vector<Pixel> genLineInterPixels(int x0, int y0, const TGAColor &c1, int x1, int y1, const TGAColor &c2);

    static vector<Pixel> genLineInterPixels(const Pixel &p1, const Pixel &p2);

    static vector<Pixel> genTriInterPixels(const Pixel &p1, const Pixel &p2, const Pixel &p3);

public:
    Image(int width, int height, const Mat4 &mtProj, const Mat4 &mtCam)
            : TGAImage(width, height, TGAImage::RGB), mtRes(makeViewportTrans(width, height) * mtProj * mtCam) {}

    void draw(const Point &point)
    {
        auto p = transform(point);
        set(p.getX(), p.getY(), point.color);
    }

    void draw(const Line &line)
    {
        auto p1 = transform(line.p1), p2 = transform(line.p2);
        draw(p1.getX(), p1.getY(), line.p1.color, p2.getX(), p2.getY(), line.p2.color);
    }

    void draw(int x0, int y0, TGAColor c0, int x1, int y1, TGAColor c1)
    {
        bool kFlag = abs(y1 - y0) > abs(x1 - x0);
        bool yFlag = (y1 - y0) * (x1 - x0) < 0;
        vector<Pixel> ps;
        if (kFlag)
        {
            if (y0 > y1)
            {
                swap(x0, x1);
                swap(y0, y1);
                swap(c0, c1);
            }
            if (yFlag)
            {
                ps = genLineInterPixels(y0, -x0, c0, y1, -x1, c1);
            } else
            {
                ps = genLineInterPixels(y0, x0, c0, y1, x1, c1);
            }
        } else
        {
            if (x0 > x1)
            {
                swap(x0, x1);
                swap(y0, y1);
                swap(c0, c1);
            }
            if (yFlag)
            {
                ps = genLineInterPixels(x0, -y0, c0, x1, -y1, c1);
            } else
            {
                ps = genLineInterPixels(x0, y0, c0, x1, y1, c1);
            }
        }

        for (auto &p: ps)
        {
            int x, y;
            if (kFlag)
            {
                y = p.x;
                x = p.y * (yFlag ? -1 : 1);
            } else
            {
                x = p.x;
                y = p.y * (yFlag ? -1 : 1);
            }
            set(x, y, p.c);
        }

    }


    void draw(const Triangle &triangle)
    {
        auto p0 = transform(triangle.p1);
        auto p1 = transform(triangle.p2);
        auto p2 = transform(triangle.p3);
        draw(p0.getX(), p0.getY(), triangle.p1.color,
             p1.getX(), p1.getY(), triangle.p2.color,
             p2.getX(), p2.getY(), triangle.p3.color);

    }

    void
    draw(int x0, int y0, const TGAColor &c0, int x1, int y1, const TGAColor &c1, int x2, int y2, const TGAColor &c2)
    {
        auto pixels = genTriInterPixels(Pixel(x0, y0, c0), Pixel(x1, y1, c1), Pixel(x2, y2, c2));
        for (auto &p: pixels)
        {
            set(p.x, p.y, p.c);
        }
    }

    void save(const char *filename)
    {
        flip_vertically();
        write_tga_file(filename);
    }
};

vector<Image::Pixel> Image::genLineInterPixels(int x0, int y0, const TGAColor &c1, int x1, int y1, const TGAColor &c2)
{
    vector<Image::Pixel> ret;
    auto f = [x0, y0, x1, y1](double x, double y)
    {
        return (y0 - y1) * x + (x1 - x0) * y + x0 * y1 - x1 * y0;
    };
    int y = y0;
    double d = f(x0 + 1, y0 + 0.5);
    double t = 1.0 / (x1 - x0);
    for (int x = x0; x <= x1; x++)
    {
        ret.emplace_back(x, y, interpolate(c1, c2, t * (x - x0)));
        if (d < 0)
        {
            ++y;
            d += (x1 - x0) + (y0 - y1);
        } else
        {
            d += (y0 - y1);
        }
    }
    return ret;
}

vector<Image::Pixel> Image::genTriInterPixels(const Image::Pixel &p0, const Image::Pixel &p1, const Image::Pixel &p2)
{
    vector<Pixel> ret;
    int xMin = min(min(p1.x, p2.x), p0.x), xMax = max(max(p1.x, p2.x), p0.x),
            yMin = min(min(p1.y, p2.y), p0.y), yMax = max(max(p1.y, p2.y), p0.y);
    auto f = [](const Pixel &p0, const Pixel &p1, int x, int y)
    {
        return (p0.y - p1.y) * x + (p1.x - p0.x) * y + p0.x * p1.y - p1.x * p0.y;
    };
    auto f01 = [&p0, &p1, &f](int x, int y) { return f(p0, p1, x, y); };
    auto f12 = [&p1, &p2, &f](int x, int y) { return f(p1, p2, x, y); };
    auto f20 = [&p2, &p0, &f](int x, int y) { return f(p2, p0, x, y); };
    auto b12 = f12(p0.x, p0.y);
    auto b01 = f01(p2.x, p2.y);
    auto b20 = f20(p1.x, p1.y);
    for (int y = yMin; y <= yMax; ++y)
    {
        for (int x = xMin; x <= xMax; ++x)
        {
            double a = f12(x, y) / static_cast<double>(b12);
            double b = f01(x, y) / static_cast<double>(b01);
            double c = f20(x, y) / static_cast<double>(b20);
            if (a > 0 && b > 0 && c > 0)
            {
                auto color = a * convertColorToVec4(p0.c) + b * convertColorToVec4(p2.c) + c * convertColorToVec4(p1.c);
                ret.emplace_back(x, y, makeColor(color));
            }
        }
    }
    return ret;
}

inline vector<Image::Pixel> Image::genLineInterPixels(const Image::Pixel &p1, const Image::Pixel &p2)
{
    return genLineInterPixels(p1.x, p1.y, p1.c, p2.x, p2.y, p2.c);
}


void testVec()
{
    Vec3 v1{1, 2, 3};
    Vec3 v2{2, 1, -1};
    Vec3 v3 = v1.cross(v2);
    cout << v3 << endl;
    cout << v3.dot(v1) << endl;
    cout << v3.dot(v2) << endl;
    cout << Vec<double, 5>(v3, 2., 2.) << endl;
    cout << Vec<double, 6>(v3, 2., 1, 'a') << endl;
}

void testMat()
{


    Mat4 mtView = makeViewportTrans(100, 100);


    cout << "View Mat:" << mtView << endl;

    Vec3 p{0, 0.5, 0.5};
    cout << Vec3(mtView * Vec4(p, 1)) << endl;

    p = {-1, -1, 0};
    cout << Vec3(mtView * Vec4(p, 1)) << endl;

    p = {1, 1, 0};
    cout << Vec3(mtView * Vec4(p, 1)) << endl;


    Mat4 mtOrtho = makeOrthographicProjectTrans(-2, -2, 2, 2, 2, -2);
    Vec3 p2{-2, -2, -2};
    cout << Vec3(mtOrtho * Vec4(p2, 1)) << endl;
    Vec3 p3{2, 2, 2};
    cout << Vec3(mtOrtho * Vec4(p3, 1)) << endl;
    Vec3 p4{1, 1, 1};
    cout << Vec3(mtOrtho * Vec4(p4, 1)) << endl;

    Vec3 eye, gaze, t;
    eye = Vec3(4, 4, 4);
    gaze = Vec3(-1, -1, -1);
    t = Vec3{1, -1, 0};

    Mat4 mtCam = makeCameraTrans(eye, gaze, t);
    Mat4 mtRes = mtView * mtOrtho * mtCam;
    cout << "mtCam: " << mtCam << endl;
    cout << "mtRes: " << mtRes << endl;
    Vec3 p1{2, 0, 2};
    cout << Vec3(mtRes * Vec4(p1, 1)) << endl;
    cout << Vec3(mtRes * Vec4(p2, 1)) << endl;
    cout << Vec3(mtRes * Vec4(p3, 1)) << endl;
    cout << Vec3(mtRes * Vec4(p4, 1)) << endl;


    Mat4 mtPer = mtView * makePerspectiveProjectTrans(-5, -5, 2, 5, 5, -2) * mtCam;
    cout << mtPer << endl;
    auto v = mtPer * Vec4(p1, 1);
    v.multiple(1 / v[3]);
    cout << Vec3(v) << endl;
    v = mtPer * Vec4(p2, 1);
    v.multiple(1 / v[3]);
    cout << Vec3(v) << endl;
    v = mtPer * Vec4(p3, 1);
    v.multiple(1 / v[3]);
    cout << Vec3(v) << endl;
    v = mtPer * Vec4(p4, 1);
    v.multiple(1 / v[3]);
    cout << Vec3(v) << endl;


}

void testImage()
{
    const TGAColor red = TGAColor(255, 0, 0, 255);
    const TGAColor green = TGAColor(0, 255, 0, 255);
    const TGAColor blue = TGAColor(0, 0, 255, 255);
    const TGAColor yellow = TGAColor(0, 128, 255, 255);

    Mat4 mtPer = makePerspectiveProjectTrans(-5, -5, -5, 5, 5, -10);

    Vec3 eye, gaze, t;
    eye = Vec3(-6, -6, 6);
    gaze = Vec3(1, 1, -1);
    t = Vec3{-1, 1, 0};
    Mat4 mtCam = makeCameraTrans(eye, gaze, t);

    Image img(300, 300, mtPer, mtCam);

    auto p1 = Point{{0, 0, 2}, red};
    auto p2 = Point{{2, 2, 0}, yellow};
    auto p3 = Point{{2, -2, 0}, blue};
    auto p4 = Point{{0, 2, 0}, green};

    img.draw(Triangle(p1, p2, p3));
    img.draw(Triangle(p1, p2, p4));
    img.draw(Triangle(p2, p3, p4));
    img.draw(Triangle(p1, p3, p4));

//    img.draw(Point({1, -1, 1}, red));
//    img.draw(Point({-1, 1, 1}, red));
//    img.draw(Point({1, 1, -1}, red));
//    img.draw(Point({1, 1, 1}, red));
//    img.draw(Point({-1, 1, -1}, red));
//    img.draw(Point({-1, -1, 1}, red));
//    img.draw(Point({1, -1, -1}, red));
//    img.draw(Point({-1, -1, -1}, red));

//    img.draw(Line(Point({-2, -2, -2}, green), Point({2, 1, 2}, red)));
//    img.draw(Line(Point({2, 1, 2}, red), Point({1, -1, 1}, yellow)));
//    img.draw(Line(Point({1, -1, 1}, yellow), Point({-2, -2, -2}, green)));

//    img.draw(Triangle(Point({-1, -1, -1}, yellow), Point({1, -1, -1}, red), Point({-1, 1, 1}, green)));

    img.save("output.tga");

}


int main()
{
//    testVec();
//    testMat();
    testImage();
    return 0;
}
