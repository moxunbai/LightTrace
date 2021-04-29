//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
     if (depth > this->maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }
    Intersection intersection = Scene::intersect(ray);
	Material *m = intersection.m;
	
	if(!intersection.happened){
		return Vector3f(0.0,0.0,0.0);
	}
	
	if(m->hasEmission()){
		return m->getEmission();
	}
	
	Intersection lightInter ;
	float pdf_light;
	Scene::sampleLight(lightInter,pdf_light);;
	
	auto wo = (-ray.direction).normalized();
	Vector3f hitPoint = intersection.coords;
    Vector3f N = (intersection.normal).normalized();
	
	auto reflect = lightInter.coords - hitPoint;
	
	auto ws = reflect.normalized();
	

    //lightInter.distance = (lightInter.coords - hitPoint ).norm();
	//bool block = (Scene::intersect(Ray(hitPoint , ws)).distance - lightInter.distance )> EPSILON;
	// bool block = (lightInter.coords - Scene::intersect(Ray(hitPoint , ws)).coords).norm()> EPSILON;
    bool block = (Scene::intersect(Ray(hitPoint , ws)).coords - lightInter.coords ).norm()> EPSILON;
	auto l_dir = Vector3f(0.0,0.0,0.0);
	if(!block){
	  l_dir = lightInter.emit* m->eval(wo,ws,N)*dotProduct(ws,N)*dotProduct(-ws,lightInter.normal.normalized())/dotProduct(reflect,reflect)/(pdf_light  );
	}
    
	auto l_indir = Vector3f(0.0,0.0,0.0);
	if(get_random_float() < RussianRoulette){
		auto wi = m->sample( wo , N).normalized();
		Ray sr(hitPoint, wi); 
		auto non_light_int = Scene::intersect(sr);
		if(non_light_int.happened && !non_light_int.m->hasEmission()){
			l_indir =  castRay(sr , depth+1 ) *  m->eval( wo ,wi , N)* dotProduct(wi , N)/m->pdf(wo , wi , N)/RussianRoulette;
		}
	}
	
	return l_dir + l_indir;
     
}