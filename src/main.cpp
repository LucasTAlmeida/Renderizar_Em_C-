#include <aurora/Math.h>
#include <aurora/Vector.h>
#include <aurora/Color.h>
#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace projeto1;




struct Camera {
     float fieldOfView;
     Film film;
     PMatrix3D worldMatrix;
    
     Camera() {}
     Camera(float fieldOfView, Film film, PMatrix3D worldMatrix) {
        this->fieldOfView = fieldOfView;
        this->film = film;
        this->worldMatrix = worldMatrix;
    }
    
     virtual void lookAt(Vector3 position, Vector3 target, Vector3 up) {
        Vector3 w = Vector3.sub(position, target).normalize();
        Vector3 u = up.cross(w).normalize();
        Vector3 v = w.cross(u);
        
        worldMatrix.set(
            u.x, v.x, w.x, position.x,
            u.y, v.y, w.y, position.y,
            u.z, v.z, w.z, position.z,
            0, 0, 0, 1.0);
    }
    virtual void transform(PMatrix3D transformation) {
        worldMatrix.preApply(transformation);
    }
    
    virtual Ray generateRay(float x, float y, Vector2 sample) {
        float scale = tan(fieldOfView * 0.5);
        
        Vector2 pixel = new Vector2();
        
        pixel.x = (2.0 * (x + 0.5 + sample.x) / film.width - 1.0) * scale * film.aspectRatio();
        pixel.y = (1.0 - 2.0 * (y + 0.5 + sample.y) / film.height) * scale;
              
        worldMatrix.mult(pixel, pixel);
        
        Vector2 origin = new Vector2(worldMatrix.m02, worldMatrix.m12);
        Vector2 direction = Vector2.sub(pixel, origin).normalize();
        
        return new Ray(origin, direction);
    }
}

struct Film {
     float width;
     float height;
    
     Film() {}
     Film(float width, float height) {
        this->width = width;
        this->height = height;
    }
    
     virtual float aspectRatio() {
        return width / height;
    }
}

struct Ray {
	Vector3 origin;
	Vector3 direction;
	
	Ray() {}
	Ray(const Vector3 & origin, const Vector3 & direction) {
		this->origin = origin;
		this->direction = direction;
	}
	
	Vector3 point(float distance) const {
		return origin + direction * distance;
	}
};

struct Intersection {
	bool hit;
	float distance;
	int index;
	
	Intersection() {
		hit = false;
		distance = PROJETO1_INFINITY;
		index = -1;
	}
	Intersection(bool hit, float distance, int index) {
		this->hit = hit;
		this->distance = distance;
		this->index = index;
	}
};

struct ShaderGlobals {
	Vector3 point;
	Vector3 normal;
	Vector2 uv;
	Vector3 tangentU;
	Vector3 tangentV;
	Vector3 viewDirection;
	Vector3 lightDirection;
	Vector3 lightPoint;
	Vector3 lightNormal;
	
	ShaderGlobals() {}
	ShaderGlobals(
			const Vector3 & point, const Vector3 & normal, const Vector2 & uv,
			const Vector3 & tangentU, const Vector3 & tangentV,
			const Vector3 & viewDirection, const Vector3 & lightDirection,
			const Vector3 & lightPoint, const Vector3 & lightNormal) {
		this->point = point;
		this->normal = normal;
		this->uv = uv;
		this->tangentU = tangentU;
		this->tangentV = tangentV;
		this->viewDirection = viewDirection;
		this->lightDirection = lightDirection;
		this->lightPoint = lightPoint;
		this->lightNormal = lightNormal;
	}
};

enum BSDFType {
	Light = 0,
	Diffuse,
	Specular,
	None
};

struct BSDF {
	BSDFType type;
	Color3 color;
	
	BSDF() {}
	BSDF(BSDFType type, const Color3 & color) {
		this->type = type;
		this->color = color;
	}
};

struct Shape {
	BSDF * bsdf;
	
	Shape() {}
	Shape(BSDF * bsdf) {
		this->bsdf = bsdf;
	}
	
	virtual bool intersects(const Ray & ray, Intersection & intersection) = 0;
	virtual void calculateShaderGlobals(
			const Ray & ray, const Intersection & intersection,
			ShaderGlobals & shaderGlobals) = 0;
	virtual float surfaceArea() = 0;
};

struct Sphere : Shape {
	Vector3 position;
	float radius;
	
	Sphere() : Shape() {}
	Sphere(const Vector3 & position, float radius, BSDF * bsdf) : Shape(bsdf) {
		this->position = position;
		this->radius = radius;
	}
	
	virtual bool intersects(const Ray & ray, Intersection & intersection) {
		Vector3 l = position - ray.origin;
		float t = l.dot(ray.direction);
		
		if (t < 0)
			return false;
			
		float d2 = l.dot(l) - t * t;
		float r2 = radius * radius;
		
		if (d2 > r2)
			return false;
		
		float dt = std::sqrt(r2 - d2);
		
		float t0 = t - dt;
		float t1 = t + dt;
		
		if (t0 > t1)
			std::swap(t0, t1);
		
		if (t0 < 0) {
			t0 = t1;
			
			if (t0 < 0)
				return false;
		}
		
		intersection.hit = true;
		intersection.distance = t0;
		
		return true;
	}
	virtual void calculateShaderGlobals(
			const Ray & ray, const Intersection & intersection,
			ShaderGlobals & shaderGlobals) {
		shaderGlobals.point = ray.point(intersection.distance);
		shaderGlobals.normal = (shaderGlobals.point - position).normalize();
		
		float theta = std::atan2(shaderGlobals.normal.x, shaderGlobals.normal.z);
		float phi = std::acos(shaderGlobals.normal.y);
		
		shaderGlobals.uv.x = theta * PROJETO1_INV_PI * 0.5;
		shaderGlobals.uv.y = phi * PROJETO1_INV_PI;
		
		shaderGlobals.tangentU.x = std::cos(theta);
		shaderGlobals.tangentU.y = 0;
		shaderGlobals.tangentU.z = -std::sin(theta);
		
		shaderGlobals.tangentV.x = std::sin(theta) * std::cos(phi);
		shaderGlobals.tangentV.y = -std::sin(phi);
		shaderGlobals.tangentV.z = std::cos(theta) * std::cos(phi);
	}
	virtual float surfaceArea() {
		return 4.0 * PROJETO1_PI * radius * radius;
	}
};

struct Scene {
	std::vector<Shape *> shapes;
	
	Scene();
	Scene(const std::vector<Shape *> & shapes) {
		this->shapes = shapes;
	}
	
	bool intersects(const Ray & ray, Intersection & intersection) {
		for (int i = 0; i < shapes.size(); i++) {
			Shape * shape = shapes[i];
			
			Intersection temp;
			shape->intersects(ray, temp);
			
			if (temp.hit && temp.distance < intersection.distance) {
				intersection.hit = temp.hit;
				intersection.distance = temp.distance;
				intersection.index = i;
			}
		}
		
		return intersection.hit;
	}
};

int main(int argc, char ** argv) {
	BSDF * bsdf = new BSDF(BSDFType::Diffuse, Color3(1.0, 1.0, 1.0));
	Shape * shape = new Sphere(Vector3(0, 0, 0), 1.0, bsdf);
	
	std::vector<Shape *> shapes;
	shapes.push_back(shape);
	
	Scene scene(shapes);
	
	Ray ray(Vector3(0, 0, 10.0), Vector3(0, 0, -1.0));
	Intersection intersection;
	
	scene.intersects(ray, intersection);
	
	std::cout << "Hit: " << intersection.hit << std::endl;
	std::cout << "Distance: " << intersection.distance << std::endl;
	std::cout << "Index: " << intersection.index << std::endl;
	
	delete bsdf;
	delete shape;
	
	return 0;
}
