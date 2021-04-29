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

bool visible(const Vector3f &srcPoint,const Vector3f &tagPoint,){
   Vector3f dir = tagPoint - srcPoint;
  bool block = (Scene::intersect(Ray(hitPoint,dir.normalized())).coords-srcPoint).norm() > EPSILON;
  return !block;
}

//  float Scene::get_random_v(){
//         std::uniform_real_distribution<float> dist(0.f, 1.f); 

//         return dist(Scene::m_rng);
//     }
	
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
     if (depth > this->maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }
    Intersection intersection = Scene::intersect(ray);

    if(!intersection.happened){
        return Vector3f(0.0,0.0,0.0);
    }
    if(intersection.m->hasEmission()){
        return intersection.m->getEmission();
    }

    Intersection lightInter;
  
    float pdf_light;
    Scene::sampleLight(lightInter,pdf_light);

    auto wo = (-ray.direction).normalized();
    Vector3f hitPoint = intersection.coords;
    Vector3f N = intersection.normal.normalized(); // normal
    Material *m = intersection.m;

    auto ret  = lightInter.coords - hitPoint;
    auto ws = ret.normalized();
    // lightInter.distance =  (lightInter.coords - hitPoint).norm();
    // bool block = ( Scene::intersect(Ray(hitPoint,ws)).distance - lightInter.distance)> EPSILON ;
    // bool block =  Scene::intersect(Ray(hitPoint,ws)).distance > lightInter.distance;
    bool block = (Scene::intersect(Ray(hitPoint,ws)).coords-lightInter.coords).norm() > EPSILON;

    Vector3f L_dir = Vector3f(0.0,0.0,0.0);
    if(!block){
          L_dir = lightInter.emit * m->eval(wo,ws,N)*dotProduct(ws,N)*dotProduct(-ws,lightInter.normal.normalized())/dotProduct(ret,ret)/(pdf_light );
    }

    Vector3f L_indir_reflect = Vector3f(0.0,0.0,0.0);
    Vector3f L_indir_trans = Vector3f(0.0,0.0,0.0);
    if(m->getType() == REFLECTION){
        Vector3f reflectionDirection = normalize(reflect(-wo, N));
        Vector3f refractionDirection = normalize(refract(-wo, N, m->ior));
         Ray srRefle = Ray(hitPoint, reflectionDirection);
         Ray srRefla = Ray(hitPoint, refractionDirection);
         float fr = 0.0f;
         fresnel(-wo,N,m->ior,fr);
        //   return L_dir +    (1-fr)*castRay(srRefla , depth+1) ;
          return L_dir +  fr*castRay(srRefle , depth+1)  +  (1-fr)*castRay(srRefla , depth+1) ;
    }else{

      if( get_random_v() < RussianRoulette){
           Vector3f wi = m->sample( wo , N).normalized();
           Ray sr = Ray(hitPoint, wi);
           Intersection nolight_inter = Scene::intersect(sr);
           if(nolight_inter.happened && !nolight_inter.m->hasEmission()){
              L_indir_reflect = castRay(sr , depth+1) * m->eval( wo ,wi , N)* dotProduct(wi , N)/m->pdf(wo , wi , N)/RussianRoulette;
           }

           
        }
        return L_dir + L_indir_reflect + L_indir_trans;
    }
    
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
        const auto camera_sample = camera->sample_wi( intersection.coords );
        if( camera_sample.we.norm()>0.0)
        {
           if(Scene::visible(intersection.coords , )){
             Vector3f  dwo = camera_sample.ref_to_pos.normalized();
             Vector3f lDir = intersection.m->eval(wo ,dwo ,intersection.normal)*coef;
             if(depth == 1){
               Vector3f epdir = lightInter.coords -  intersection.coords;
               lDir = lDir*dotProduct(-wo,intersection.normal)/dotProduct(epdir,epdir);
             }
              const real pixel_x = camera_sample.film_coord.x
                                                    * scene.width;
              const real pixel_y = camera_sample.film_coord.y
                                                    * scene.height;

              camera->apply_image_filter(Vector2f(scene.width,scene.height) ,0.5 ,Vector2f(pixel_x,pixel_y) ,image , lDir);
           }
        }
        if(depth == 1){
           Vector3f epdir = lightInter.coords -  intersection.coords;
           lDir = lDir*dotProduct(-wo,intersection.normal)/dotProduct(epdir,epdir);
        }

     }

}