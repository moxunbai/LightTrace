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

bool Scene::visible(const Vector3f &srcPoint,const Vector3f &tagPoint) const{
   Vector3f dir = tagPoint - srcPoint;
  Intersection _inte = Scene::intersect(Ray(srcPoint,dir.normalized()));
  if(!_inte.happened){
      return  true;
  }else{
      return  false;
  }

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

void Scene::lightTracing(std::vector<Vector3f> *image) const
{

    Intersection lightInter;
    float pdf_light;

    Scene::sampleLight(lightInter,pdf_light);

    Camera *camera = this->camera;


    Vector3f wo = lightInter.m->sample( Vector3f(0.0) , lightInter.normal);
    float l_pdf_dir = lightInter.m->pdf(Vector3f(0.0)  , wo , lightInter.normal);
//Vector3f wo = Vector3f(0.0);

    wo=wo.normalized();
    Vector3f hitPoint = lightInter.coords;
    Vector3f N = lightInter.normal;
    Material *m = lightInter.m;

    Vector3f coef = lightInter.emit   /pdf_light/l_pdf_dir ;
    Ray ray = Ray(lightInter.coords, wo);
    for(int depth = 0; depth <= this->maxDepth; ++depth)
     {

       auto camera_sample = camera->sample_wi( hitPoint );
        if( camera_sample.we.norm()>0.0)
        {
           if(Scene::visible(hitPoint ,camera_sample.pos_on_cam )){
             Vector3f  dwo = camera_sample.ref_to_pos.normalized();
             Vector3f lDir =   coef*camera_sample.we* std::abs(dotProduct(dwo,N));
             if(depth >0 ){
               lDir =  m->eval(dwo ,-wo ,N)*lDir;
               /*if(depth == 1){
                 Vector3f epdir = lightInter.coords -  hitPoint;
                 lDir = lDir*dotProduct(-wo,N)/dotProduct(epdir,epdir);
                }*/
//                if(depth >1 && m->getType() == DIFFUSE){
//                   std::cout << "Render PPPPPPPPPPP: "<<camera_sample.we<<"---"<< m->eval(-wo ,dwo ,N)<<"\n";
//                }
             }
              const float pixel_x = camera_sample.film_coord.x
                                                    * this->width;
              const float pixel_y = camera_sample.film_coord.y
                                                    * this->height;
               Vector2f pixel_range(this->width-1,this->height-1);
               Vector2f pixel_sample(pixel_x,pixel_y);
//               std::cout << "Render DDDDDDDDD:  "<<camera_sample.film_coord.x<<"-----"<<camera_sample.film_coord.y <<"\n";
               camera->apply_image_filter( pixel_range ,0.5f ,pixel_sample ,image , lDir);
           }
        }

        Intersection intersection = Scene::intersect(ray);
        if(!intersection.happened){
                return  ;
        }
        if(intersection.m->hasEmission()){
                return  ;
         }

         wo = ray.direction.normalized();
         hitPoint = intersection.coords;
         m = intersection.m;
         N = intersection.normal;
        Vector3f wi  ;
        float PD;

        if(depth == 0){
           Vector3f epdir = lightInter.coords -  hitPoint;
           float distanc = dotProduct(epdir,epdir);
           coef = coef*dotProduct(-wo,N)/distanc ;
        }
        if(m->getType() == MICROFACET){
           Vector3f f  = m->sample_F( -wo , N,wi ,PD) ;
           if(PD>0){
             wi = wi.normalized();
             coef = coef* f*std::abs(dotProduct(wi , N)) /PD;
//             coef = Vector3f::Min(Vector3f::Max( coef,Vector3f(0.0f) ),Vector3f(1));
           }
        }else{
                wi = m->sample( -wo , N).normalized();
                PD=m->pdf(-wo , wi , N);
                if(PD>0){
                  coef = coef*  m->eval( -wo ,wi , N)*std::abs(dotProduct(wi , N)) /PD;

                }

        }
//        if(depth > 0){
//          wo = wi;
//        }
        ray = Ray(hitPoint, wi);
     }

}