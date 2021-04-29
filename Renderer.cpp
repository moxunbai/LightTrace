//
// Created by goksu on 2/25/20.
//

#include <thread>
#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include<unistd.h>


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

// const float EPSILON = 0.00001;
const float EPSILON = 0.0001;

int thread_num =200;
std::atomic<int> proc( 0);

int  a = 784*784;
 

void worker(int j,float si,float scale ,const Scene &scene ,  std::vector<Vector3f>  &fbuffer ){
   Vector3f eye_pos(278, 273, -800);
   
   int mc = std::ceil( (float)scene.height/thread_num);
   int start =j*mc;
   int spp = 122;
//    std::cout << "start  :"<<start <<"  mc:"<<mc <<"  j=" << j << std::endl;

//   std::cout << "Thread fun  run j=:"<< j << " ---" <<start <<" ==== " <<  mc << std::endl;
   for(int k = start ;k<start + mc && k <scene.height; ++k ){
       
        for (uint32_t i = 0; i < scene.width; ++i) {
                    // generate primary ray direction
                    float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                            si;
                    float y = (1 - 2 * (k + 0.5) / (float)scene.height) * scale;
                    
                    int  idx = k*scene.width+i;
                    
                    if(idx>= a -1){
                        // std::cout << "too lage  :"<<idx  << std::endl;
                    }
                   

                    // std::cout << "Thread fun  run j=:"<< j << " ---" <<cc  << std::endl;
                    Vector3f dir = normalize(Vector3f(-x, y, 1));
                    for (int q = 0; q < spp; q++){
                        // std::cout << "Thread fun  run k=:"<<(k*scene.width+i)   << std::endl;
                        fbuffer[idx] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
                    }
                     
            }
       proc++;   
   }
    
}

void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height) ;
    // framebuffer.resize(scene.width * scene.height);
    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;

    float si= imageAspectRatio * scale;
 
     
    // change the spp value to change sample ammount
     
    for (uint32_t j = 0; j <  thread_num  ; ++j) {
        
        std::thread t(worker , j ,si ,scale, std::ref(scene) , std::ref(framebuffer)  ) ;
      
        t.detach();
    }

    while(proc < scene.height-1){
        UpdateProgress(proc / (float)scene.height);

        sleep(5);
    }
    

    std::cout << "Thread fun  run proc=:"<< proc   << std::endl;

    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}

std::vector<Vector3f> render_forward(){

}
std::vector<Vector3f> render_backward(){
   std::mutex reporter_mutex;
   std::atomic<int> next_task_id = 0;
   const int worker_count = 100;
   const int particle_task_count = 2000;
   const int particles_per_task = 100;
   std::atomic<uint64_t> total_particle_count = 0;

   std::vector<std::thread> threads;
   std::vector<std::vector<Vector3f>> images;
   threads.reserve(worker_count);
   images.resize(worker_count, std::vector<Vector3f>(scene.width * scene.height));
   auto backward_func =
            [&](  std::vector<Vector3f> *image, int i)
        {

            for(;;)
            {

                const int task_id = next_task_id++;
                if(task_id >= particle_task_count)
                    break;

                int task_particle_count = 0;
                for(int j = 0; j <  particles_per_task; ++j)
                {
                    ++task_particle_count;
                    scene.castLightRay(image)
                }
                total_particle_count += task_particle_count;

                const real percent = real(100) * (task_id + 1)
                                   /  particle_task_count;
                std::lock_guard lk(reporter_mutex);
                UpdateProgress( percent);
            }
        };
        for(int i = 0; i < worker_count; ++i)
        {

            threads.emplace_back(backward_func,  &images[i], i);
        }

        for(auto &t : threads)
            t.join();
        const real scale = scene.width * scene.height/ static_cast<real>(total_particle_count);
        std::vector<Vector3f> ret(scene.width * scene.height);
        for(auto &img : images){
          for(int i=0;i<scene.width * scene.height;i++)
          ret[i] = ret[i] + img[i];

        }
        ret = ret * scale;

}