#include <aurora/Utility.h>
#include <aurora/Math.h>
#include <aurora/Vector.h>
#include <aurora/Color.h>
#include <aurora/Image.h>
#include <aurora/Matrix.h>
#include <aurora/Global.h>
         
using namespace std;

int main(int argc, char ** argv) {
    
    enum BSDFType
    {
    int ligth=0;
    int difuse=1;
    int specular=2;
    int nome=3;
    }
    
	return 0;
}


public class Scene {
    public ArrayList<Shape> shapes;
    
    public Scene() {}
    public Scene(ArrayList<Shape> shapes) {
        this.shapes = shapes;
    }
    
    public Intersection intersects(Ray ray) {
        Intersection intersection = new Intersection();
        
        for (int i = 0; i < shapes.size(); i++) {
            Shape shape = shapes.get(i);
            Intersection temp = shape.intersects(ray);
            
            if (temp.hit && temp.distance < intersection.distance) {
                intersection = temp;
                intersection.index = i;
            }
        }
        
        return intersection;
    }
}
 typedef struct BSDF BSDFType{
         
        public Color3 color;
         
    public BSDF() {}
    public BSDF sample(BSDF BSDFType, color Color3);
}


public class Shape{
   
    public BSDF bsdf;
           
    public Shape() {}
    public Shape(BSDF bsdf) {
        
        this.bsdf = bsdf;      
    }
    
    public Intersection intersects(Ray ray);
    public ShaderGlobals calculateShaderGlobals(Ray ray, Intersection intersection);
    public float surfaceArea();
}
public Intersection intersects(Ray ray) {
        Vector3 d = Vector3.sub(position, ray.origin);
        float t = d.dot(ray.direction);
        
        if (t < 0)
            return new Intersection();
        
        float d2 = d.dot(d) - t * t;
        float r2 = radius * radius;
        
        if (d2 > r2)
            return new Intersection();
        
        float dt = sqrt(r2 - d2);
        
        return new Intersection(true, t - dt, -1);
    }


public ShaderGlobals calculateShaderGlobals(Ray ray, Intersection intersection) {
        Vector3 point = ray.point(intersection.distance);
        Vector3 normal = Vector3.sub(point, position).normalize();
        
        float theta = atan2(normal.x, normal.z);
        float phi = acos(normal.y);
        
        Vector3 uv = new Vector3((theta * INVERSE_PI + 1.0) * 0.5, phi * INVERSE_PI);
        
        float st = sin(theta);
        float sp = sin(phi);
        float ct = cos(theta);
        float cp = cos(phi);
        
        Vector3 tangentU = new Vector3(ct, 0, -st);
        Vector3 tangentV = new Vector3(st * cp, -sp, ct * cp);
        
        Vector3 viewDirection = Vector3.mult(ray.direction, -1.0);
        
        return new ShaderGlobals(point, normal, uv, tangentU, tangentV, viewDirection, null, null, null);
    }
    
    public float surfaceArea() {
        return 4.0 * PI * radius * radius;
    }
}




public class Sphere extends Shape {
    public Vector3 position;
    public float radius;
    
    public Sphere() {}
    public Sphere(Vector3 position, float radius, BSDF bsdf) {
        this.position = position;
        this.radius = radius;
    }
    

public class Ray {
     Vector3 origin;
     Vector3 direction;
    
    public Ray() {}
    public Ray(Vector3 origin, Vector3 direction) {
        this.origin = origin;
        this.direction = direction;
    }
    
    public Vector3 point(float distance) {
        return Vector3.add(origin,Vector3.mult(direction, distance));
    }
}


public class Intersection {
    public boolean hit;
    public float distance;
    public int index;
    
    public Intersection() {
    }
    public Intersection(boolean hit, float distance, int index) {
        this.hit = hit;
        this.distance = distance;
        this.index = index;
    }
}

public class ShaderGlobals {
    public Vector3 point;
    public Vector3 normal;
    public Vector2  uv;
    public Vector3 tangentU;
    public Vector3 tangentV;
    public Vector3 viewDirection;
    public Vector3 lightDirection;
    public Vector3 lightPoint;
    public Vector3 lightNormal;
    
    public ShaderGlobals() {}
    public ShaderGlobals(
        Vector3 point, Vector3 normal, Vector2 uv, Vector3 tangentU, Vector3 tangentV,
        Vector3 viewDirection, Vector3 lightDirection, Vector3 lightPoint, Vector3 lightNormal) {
        this.point = point;
        this.normal = normal;
        this.uv = uv;
        this.tangentU = tangentU;
        this.tangentV = tangentV;
        this.viewDirection = viewDirection;
        this.lightDirection = lightDirection;
        this.lightPoint = lightPoint;
        this.lightNormal = lightNormal;
    }
}


