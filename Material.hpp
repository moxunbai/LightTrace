//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { DIFFUSE, MICROFACET};

 

class Material{
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    float fresnel(const Vector3f &v, const Vector3f &h, const float &ior ) const
    {
        float cosTheta = dotProduct(v,h);
	    if (cosTheta<=0) {
		  Vector3f vInLowIOR =  refract(-v, -h, ior);
		  if (vInLowIOR.norm()==0)
			return 1.f;

		  cosTheta = dotProduct(vInLowIOR,h);
	    }

	   float R0 = pow((ior - 1) / (ior + 1), 2);

	   return R0 + (1 - R0) * pow((1 - cosTheta), 5);
    }

   float fresnel_A5(const Vector3f &I, const Vector3f &N, const float &ior)
   {
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    if (cosi > 0) {  std::swap(etai, etat); }
    // Compute sini using Snell's law
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
    // Total internal reflection
    if (sint >= 1) {
        return 1;
    }
    else {
        float cost = sqrtf(std::max(0.f, 1 - sint * sint));
        cosi = fabsf(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp) / 2;
    }
    // As a consequence of the conservation of energy, transmittance is given by:
    // kt = 1 - kr;
   }

    float getGGX_D( const Vector3f& N, const Vector3f& harf)
    {
        float m = Roughness;
        float cosTheta = dotProduct(N, harf);
        if(cosTheta<0){
            return 0.f;
        }
        float cosTheta2 = cosTheta*cosTheta;
        float tanTheta2 = 1/cosTheta2 - 1;
        float root = m/(cosTheta2 * (m*m-tanTheta2));
        return root*root/M_PI;
    }
    float smithG(const Vector3f& N, const Vector3f& wi, const Vector3f& wo, const Vector3f& harf){
        float m = Roughness;
        if(dotProduct(wi,N)*dotProduct(wi,harf)<=0 || dotProduct(wo,N)*dotProduct(wo,harf)<=0){
            return 0.f;
        }
         return smith_G1(wi,N,harf,m)*smith_G1(wo,N,harf,m);
    }

     
    

    float smith_G1( const Vector3f& v ,const Vector3f& N ,  const Vector3f& harf ,float m)
    {
        float vn = dotProduct(v, N);
        float tanTheta = fabsf(sqrtf(1/(vn*vn)-1));
        
        if(dotProduct(v, harf)*vn<=0){
             return 0.0f;
        }
        if(tanTheta == 0.0f){
            return 1.0f;
        }
        float root = m*tanTheta;
        return 2.0f/(1.0f+sqrtf(1.0f+root*root));
       
    }
    float getShadowingMT(const Vector3f& N, const Vector3f& wi, const Vector3f& wo, const Vector3f& harf)
    {
        float v1 = 2 * dotProduct(N, harf) * dotProduct(N, wo) / (dotProduct(wo, harf));
        float v2 = 2 * dotProduct(N, harf) * dotProduct(N, wi) / (dotProduct(wi, harf));

        return std::min(1.0f,std::min(v1,v2));
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // inline Vector3f sample_F(const Vector3f &wo, const Vector3f &N ,    float &Pd );
     inline Vector3f sample_F(const Vector3f &wo, const Vector3f &N ,  Vector3f &wi,   float &Pd );
    

    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}
 

Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){

    switch(m_type){
        case DIFFUSE:
        {

            // uniform sample on the hemisphere
            float x_1 = get_random_v(), x_2 = get_random_v();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;

            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);

            return toWorld(localRay, N);
            
            break;
        }
        case MICROFACET:
        {

           float x_1 = get_random_v(), x_2 = get_random_v();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            break;
        }
    }
}

 
Vector3f Material::sample_F(const Vector3f &wo, const Vector3f &N ,  Vector3f &wi,  float & Pd )
{
    // uniform sample on the hemisphere
            float Xi1 = get_random_v(), Xi2 = get_random_v();
            float alpha = Roughness;
			// theta
	        const auto cos2Theta = (1 - Xi1) / ((alpha*alpha - 1)*Xi1 + 1);
	        const auto cosTheta = sqrt(cos2Theta);
	        const auto sinTheta = sqrt(1 - cos2Theta);
			
			// phi
	        const auto phi = 2 * M_PI * Xi2;
	
            Vector3f localRay(sinTheta*std::cos(phi), sinTheta*std::sin(phi), cosTheta);
            Vector3f h =  toWorld(localRay, N);
            // float fr = 0.0f;
            // fresnel(-wo, h, this->ior ,fr);
            float fr = fresnel(wo, h, this->ior);
            // float fr = fresnel_A5(-wo, h, this->ior);
	        bool isReflect = get_random_v() < fr;
	        if (isReflect) {
				  wi =  reflect(-wo, h);
                  
		        if (dotProduct(wo , N )*dotProduct(wi , N )<=0) {
			    //    std::cout << "sample =0: \n";
                    Pd = 0 ;
			        return Vector3f(0.f);
		        }
                float Dh = getGGX_D(N , h);
                Pd = Dh*dotProduct(h , N )/(4.f*fabs(dotProduct(wo,h))) *fr;
                float bsdf = fr * Dh * smithG(N , wi, wo, h) / fabsf(4* dotProduct(N,wi)* dotProduct(N,wo));
                // return  Vector3f(0.f);   
                //  Vector3f Ks = Vector3f(1-Kd.x, 1-Kd.y, 1-Kd.z);
				return Kd  *bsdf;
			}else{
				float etai = 1.f, etat = ior;  
				 wi =  refract(-wo, h, ior);  
		        if (dotProduct(wo , N )>0.f) {
			    //   wi =  refract(-wo, h, etai / etat);
		        }
		        else {
			      
			    //   wi =  refract(-wo, -h, etai / etat);
                  std::swap(etai, etat);
		        }

		        if (dotProduct(wo , N )*dotProduct(wi , N )>0.f || wi.norm()==0) {
		         Pd=0.f;
			     return Vector3f(0.f);
		        }
                float Dh = getGGX_D(N ,h);
                float HoWo = dotProduct(h,wo);
	        	float HoWi = dotProduct(h,wi);
	        	float sqrtDenom = etai * HoWo + etat * HoWi;
                float dwh_dwi = (etat * etat * fabsf(HoWi)) / (sqrtDenom * sqrtDenom);
                Pd = Dh*dotProduct(h , N )*dwh_dwi*(1-fr);
				 

		        float Gwowih = smithG(N , wi, wo, h);
		        float factor = fabsf(HoWo * HoWi / (dotProduct(N,wi)* dotProduct(N,wo)));
		        //  float a = ((1 - fr) * Dh * Gwowih * etat * etat);
		        float bsdfVal = factor * ((1 - fr) * Dh * Gwowih * etat * etat) /
		        	(sqrtDenom * sqrtDenom);
                // if(bsdfVal<1 && bsdfVal>0){
                    // std::cout <<" =====bsdfVal==="<<bsdfVal<<"Pd--"<<Pd <<" \n";
                // }
                    // std::cout <<"sqrtDenom:"<< sqrtDenom<<"factor:"<<factor<< " \n";
                // return  Vector3f(0.f);   
				return Kd*bsdfVal;
			}
}


float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
        case MICROFACET:
        {

            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
    }
}

Vector3f Material::eval(const Vector3f &wo, const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse   model
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
        case MICROFACET:
        {
            float cosalpha = dotProduct(N, wo);
            float cosab = dotProduct(N, wi);
            if (cosalpha*cosab > 0.0f) {

                Vector3f harf = (wi + wo).normalized();
                // if(dotProduct(N, harf)<0){
                //     harf = -harf;
                // }
                float kr = fresnel(wi, harf, this->ior);

                 float distr = getGGX_D(N ,  harf);
                float g = smithG(N ,  wi, wo, harf);
                // float distr = getDist(N, harf);
                // float g = getShadowingMT(N,wi, wo, harf);
               float fr = kr * distr * g / fabsf(4* dotProduct(N,wi)* dotProduct(N,wo));
                

                Vector3f Ks = Vector3f(1-Kd.x, 1-Kd.y, 1-Kd.z);
                // Vector3f result =   fr;
                Vector3f result =Kd *fr;
                return result;
            }else {
                 
                Vector3f harf = Vector3f(0.0,0.0,0.0);
                
                float etai=1;
                float etao=this->ior;
                if(cosalpha<0.0f){
                    std::swap(etai, etao);
                }
                harf =-(etai * wo + etao * wi).normalized();
                
                 float kr = fresnel(wi, harf, this->ior);
                 
                 
                 float distr = getGGX_D(N , harf);
                float g = smithG(N , wi, wo, harf);

                float HoWo = dotProduct(harf,wo);
	        	float HoWi = dotProduct(harf,wi);
                float sqrtDenom = etai * HoWo + etao * HoWi;

                float factor = fabsf(HoWo * HoWi / (dotProduct(N,wi)* dotProduct(N,wo)));
            //    if(dotProduct(N, harf)<0){
            //         harf = -harf;
            //     }
                float bsdfVal = factor * ((1 - kr) * distr * g * etao * etao) /(sqrtDenom * sqrtDenom);
                // float a1=fabs(dotProduct(wi,harf))*fabs(dotProduct(wo,harf));
                // float a2 =fabs(cosab)*fabs(cosalpha);
                // float a3 = etai*dotProduct(wi,harf)+etao*dotProduct(wo,harf);

                // float fr =(a1/a2)*etao*etao* (1-kr) * distr * g / (a3*a3);
                 return Kd*bsdfVal;
            }
            
            break;
            


        }
    }
}

#endif //RAYTRACING_MATERIAL_H
