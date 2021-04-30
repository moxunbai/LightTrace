//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


// std::random_device dev;
// std::mt19937 Scene::m_rng(dev());


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}
void Scene::setCamera(Camera *camera) {
     this->camera = camera;
    }
void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p =  get_random_v() * emit_area_sum;
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

bool Scene::visible(const Vector3f &srcPoint,const Vector3f &tagPoint){
   Vector3f dir = tagPoint - srcPoint;
  bool block = (Scene::intersect(Ray(srcPoint,dir.normalized())).coords-srcPoint).norm() > EPSILON;
  return !block;
}

//  float Scene::get_random_v(){
//         std::uniform_real_distribution<float> dist(0.f, 1.f); 

//         return dist(Scene::m_rng);
//     }
	
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    return Vector3f(0.f);
    
}

void Scene::lightTracing(std::vector<Vector3f> *image)
{
    Intersection lightInter;
    float pdf_light;
    Scene::sampleLight(lightInter,pdf_light);
    const Camera *camera = this->camera;
    Vector3f wo = lightInter.m->sample( Vector3f(0.0) , lightInter.normal).normalized();

    Vector3f coef =lightInter.emit  *dotProduct(wo,lightInter.normal) ;
    Ray ray = Ray(lightInter.coords, wo);
    for(int depth = 1; depth <= this->maxDepth; ++depth)
     {
        Intersection intersection = Scene::intersect(ray);
        if(!intersection.happened){
                return  ;
        }
        if(intersection.m->hasEmission()){
                return  ;
         }
       auto camera_sample = camera->sample_wi( intersection.coords );
        if( camera_sample.we.norm()>0.0)
        {
           if(Scene::visible(intersection.coords ,camera_sample.pos_on_cam )){
             Vector3f  dwo = camera_sample.ref_to_pos.normalized();
             Vector3f lDir = intersection.m->eval(wo ,dwo ,intersection.normal)*coef;
             if(depth == 1){
               Vector3f epdir = lightInter.coords -  intersection.coords;
               lDir = lDir*dotProduct(-wo,intersection.normal)/dotProduct(epdir,epdir);
             }
              const float pixel_x = camera_sample.film_coord.x
                                                    * this->width;
              const float pixel_y = camera_sample.film_coord.y
                                                    * this->height;

               camera->apply_image_filter(Vector2f(this->width,this->height) ,0.5f ,Vector2f(pixel_x,pixel_y) ,image , lDir);
           }
        }
        if(depth == 1){
           Vector3f epdir = lightInter.coords -  intersection.coords;
           coef = coef*dotProduct(-wo,intersection.normal)/dotProduct(epdir,epdir)*dotProduct(epdir,epdir) ;
        }
        Material *m = intersection.m;
        Vector3f wi  ;
        float PD;
        if(m->getType() == MICROFACET){
           Vector3f f  = m->sample_F( -wo , intersection.normal,wi ,PD) ;
           if(PD>0){
             wi = wi.normalized();
             coef = coef* f*std::abs(dotProduct(wi , intersection.normal)) /PD;

           }
        }else{
                wi = m->sample( -wo , intersection.normal).normalized();
                PD=m->pdf(-wo , wi , intersection.normal);
                if(PD>0){
                  coef = coef*  m->eval( -wo ,wi , intersection.normal)*std::abs(dotProduct(wi , intersection.normal)) /PD;
                }
        }
        ray = Ray(intersection.coords, wi);
     }

}