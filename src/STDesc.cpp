#include "include/STDesc.h"
#include "include/svpng.h"
#include <pcl/io/pcd_io.h>
#include <math.h>
void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size) {
  int intensity = rand() % 255;
  if (voxel_size < 0.01) {
    return;
  }
  std::unordered_map<VOXEL_LOC, M_POINT> voxel_map;
  uint plsize = pl_feat.size();

  for (uint i = 0; i < plsize; i++) {
    pcl::PointXYZI &p_c = pl_feat[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = voxel_map.find(position);
    if (iter != voxel_map.end()) {
      iter->second.xyz[0] += p_c.x;
      iter->second.xyz[1] += p_c.y;
      iter->second.xyz[2] += p_c.z;
      iter->second.intensity += p_c.intensity;
      iter->second.count++;
    } else {
      M_POINT anp;
      anp.xyz[0] = p_c.x;
      anp.xyz[1] = p_c.y;
      anp.xyz[2] = p_c.z;
      anp.intensity = p_c.intensity;
      anp.count = 1;
      voxel_map[position] = anp;
    }
  }
  plsize = voxel_map.size();
  pl_feat.clear();
  pl_feat.resize(plsize);

  uint i = 0;
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) {
    pl_feat[i].x = iter->second.xyz[0] / iter->second.count;
    pl_feat[i].y = iter->second.xyz[1] / iter->second.count;
    pl_feat[i].z = iter->second.xyz[2] / iter->second.count;
    pl_feat[i].intensity = iter->second.intensity / iter->second.count;
    i++;
  }
}

void test_rgb(int xsize, int ysize) {
    unsigned char rgb[xsize * ysize * 3], *p = rgb;
    unsigned x, y;

    FILE *fp = fopen("/home/shinva/mappingData/depthrgb.png", "wb");
    for (y = 0; y < ysize; y++)
        for (x = 0; x < xsize; x++) {
            *p++ = 255;    /* R */
            *p++ = 255;    /* G */
            *p++ = 255;                 /* B */
    }
    svpng(fp, xsize, ysize, rgb, 0);
    fclose(fp);
}

void down_sampling_voxel_png(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size, int minPoints, int minVcnt,
                          int zeroVcnt, float isolateRatio,bool onlyGrid,
                          int searchR, std::string resultPath) {
  int intensity = rand() % 255;
  if (voxel_size < 0.01) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI> points_sv = pl_feat;
  std::unordered_map<VOXEL_LOC, M_POINT> voxel_map;
  uint plsize = pl_feat.size();

  for (uint i = 0; i < plsize; i++) {
    pcl::PointXYZI &p_c = pl_feat[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = voxel_map.find(position);
    if (iter != voxel_map.end()) {
      iter->second.xyz[0] += p_c.x;
      iter->second.xyz[1] += p_c.y;
      iter->second.xyz[2] += p_c.z;
      iter->second.intensity += p_c.intensity;
      iter->second.count++;
    } else {
      M_POINT anp;
      anp.xyz[0] = p_c.x;
      anp.xyz[1] = p_c.y;
      anp.xyz[2] = p_c.z;
      anp.intensity = p_c.intensity;
      anp.count = 1;
      voxel_map[position] = anp;
    }
  }
  plsize = voxel_map.size();
  pl_feat.clear();
  pl_feat.resize(plsize);

  uint i = 0;
  
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) {
    pl_feat[i].x = iter->second.xyz[0] / iter->second.count;
    pl_feat[i].y = iter->second.xyz[1] / iter->second.count;
    pl_feat[i].z = iter->second.xyz[2] / iter->second.count;
    pl_feat[i].intensity = iter->second.intensity / iter->second.count;
    i++;
  }

  i = 0;
  int64_t xMax,xMin,yMax,yMin,zMax,zMin;
  xMax=xMin=yMax=yMin=zMax=zMin=0;
  std::unordered_map<VOXEL_LOC, PNG_POINT> png_map;
  int maxVcnt = 0;
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) {
    //if (iter->first.viewed) continue;
    //iter->first.viewed = true;
    int64_t xcoord = iter->first.x;
    int64_t ycoord = iter->first.y;
    VOXEL_LOC position(xcoord, ycoord, 0);
    auto pngiter = png_map.find(position);
    if (pngiter != png_map.end()) {
      if (iter->second.count > minPoints) {  // must contain enough points
      pngiter->second.vcnt++;
      pngiter->second.x += iter->second.xyz[0] / iter->second.count;
      pngiter->second.y += iter->second.xyz[1] / iter->second.count;
      pngiter->second.z += iter->second.xyz[2] / iter->second.count;
      
      maxVcnt = (pngiter->second.vcnt > maxVcnt)?pngiter->second.vcnt:maxVcnt;
      if (iter->second.xyz[2] < 1.2)
      pngiter->second.belongGround = true;
      }
    } else {
      PNG_POINT anp;
      anp.x = iter->second.xyz[0] / iter->second.count;
      anp.y = iter->second.xyz[1] / iter->second.count;
      anp.z = iter->second.xyz[2] / iter->second.count;
      anp.vcnt = 1;
      if (iter->second.xyz[2] < 1.2)
      anp.belongGround = true;
      png_map[position] = anp;
    }
    xMax = (iter->first.x > xMax)?iter->first.x:xMax;
    xMin = (iter->first.x < xMin)?iter->first.x:xMin;

    yMax = (iter->first.y > yMax)?iter->first.y:yMax;
    yMin = (iter->first.y < yMin)?iter->first.y:yMin;

    zMax = (iter->first.z > zMax)?iter->first.z:zMax;
    zMin = (iter->first.z < zMin)?iter->first.z:zMin;
    i++;
  }
  // gray = 0.299*r + 0.587*g + 0.114*b;
  // test_rgb(xMax-xMin, yMax-yMin);
  printf("xMax=%ld   xMin = %ld, yMax=%ld yMin=%ld\n", xMax, xMin, yMax,yMin);
  
  int xsize = xMax-xMin;
  int ysize = yMax-yMin;
  printf("png_map size=%ld   ysize = %d, xsize=%d\n", png_map.size(), ysize, xsize);
  
  unsigned x, y;
  int searchRadius = searchR;
  int searchR5 = 5;
  for (y = ysize; y >0 ; y--)
    for (x = 0; x < xsize; x++) {
      int64_t xcoord = x+xMin;
      int64_t ycoord = y+yMin;
      VOXEL_LOC position(xcoord, ycoord, 0);
      auto pngiter = png_map.find(position);
      if (pngiter != png_map.end()) {
        pngiter->second.x = pngiter->second.x/pngiter->second.vcnt;
        pngiter->second.y = pngiter->second.y/pngiter->second.vcnt;
        pngiter->second.z = pngiter->second.z/pngiter->second.vcnt;
      }
  }

  

  float lsize= ceil(1.0/voxel_size);
  printf("lsize=%f   voxel_size = %f searchRadius=%d\n", lsize, voxel_size,searchRadius);
  int totalvoxels = 0;
  int l1 , l2,l3 ,l4,l5,l6,l7,l8,l9,l10;
  l1 = l2 =l3 =l4=l5=l6=l7=l8=l9=l10 = 0;
  
  for (y = ysize; y >0 ; y--)
    for (x = 0; x < xsize; x++) {
      int64_t xcoord = x+xMin;
      int64_t ycoord = y+yMin;
      VOXEL_LOC position(xcoord, ycoord, 0);
      auto pngiter = png_map.find(position);
      bool modified = false;
      PNG_POINT cp;
      PNG_POINT lp, rp;

      if (pngiter != png_map.end()) {
        int h = pngiter->second.vcnt;
        totalvoxels++;
        if (h>0 && h<=lsize)
        l1++;
        else if (h>lsize && h<=2*lsize)
        l2++;
        else if (h>2*lsize && h<=3*lsize)
        l3++;
        else if (h>3*lsize && h<=4*lsize)
        l4++;
        else if (h>4*lsize && h<=5*lsize)
        l5++;
        else if (h>5*lsize && h<=6*lsize)
        l6++;
        else if (h>6*lsize && h<=7*lsize)
        l7++;
        else if (h>7*lsize && h<=8*lsize)
        l8++;
        else if (h>8*lsize)
        l9++;
      }
      

      if (pngiter != png_map.end()&&
      (pngiter->second.vcnt > minVcnt)) {
        
        //printf(" index %ld %ld  coord %f %f\n",pngiter->first.x, pngiter->first.y,pngiter->second.x,pngiter->second.y);
        cp.x = pngiter->second.x;
        cp.y = pngiter->second.y;
        cp.d = sqrt(cp.x*cp.x + cp.y*cp.y);

          bool lfind = false;
          bool rfind = false;
          //for (int l = searchRadius-1; l>1; l--) {   // cong zui wai ceng kai shi cha zhao zuo you liang ce shuju  
          for (int l = -searchR5-1; l<searchR5; l++) {   // cong zui wai ceng kai shi cha zhao zuo you liang ce shuju 
            bool rif = false;  
            bool lif = false;
            float stheta =asin(1.0/l);
            //for (float m = 0; m<=M_PI*2; m+=stheta) {
            for (float m = -searchR5; m<searchR5; m++) {
              //int64_t xinc = l*cos(m);
              //int64_t yinc = l*sin(m);
              int64_t xinc = l;
              int64_t yinc = m;
              //printf("search angle m= %f inc angle=%f  xyinc %ld  %ld\n",m,stheta, xinc, yinc);
              VOXEL_LOC pNeighbor(xcoord + xinc, ycoord +yinc, 0);
              auto Neighbor = png_map.find(pNeighbor);
              if (Neighbor != png_map.end() &&
              (Neighbor->second.vcnt >= zeroVcnt)) {
                  //if (Neighbor->second.lineType == 1) continue;
                  PNG_POINT np;
                  np.x = Neighbor->second.x;
                  np.y = Neighbor->second.y;
                  
                  //printf(" index %ld %ld  coord %f %f\n",pngiter->first.x, pngiter->first.y,pngiter->second.x,pngiter->second.y);
                  //if ((cp.x*np.y-np.x*cp.y)>0 && (!rif)) {
                  if ((cp.x*np.y-np.x*cp.y)>0) {
                    rp.x += (np.x-cp.x);
                    rp.y += (np.y-cp.y);
                    //rp.x = (np.x);
                    //rp.y = (np.y);
                    //float dist = sqrt(np.x*np.x + np.y*np.y);
                    //rp.d += (dist-cp.d);
                    rp.count++;
                    //if (fabs(rp.d)>200) {
                    //  printf("\n 1111  neighbor delta index %ld %ld  coord [%f %f] [%f %f] rpd [%f  %f  =%f]\n",
                    //  Neighbor->first.x- pngiter->first.x, Neighbor->first.y-pngiter->first.y,
                    //  Neighbor->second.x, Neighbor->second.y, pngiter->second.x, pngiter->second.y,dist,cp.d, rp.d);
                    //  printf("search angle m= %f inc angle=%f  xyinc %ld  %ld\n",m,stheta, xinc, yinc);
                    //}
                    
                    rif = true;
                    rfind = true;
                  }
                  // else if ((cp.x*np.y-np.x*cp.y)<0 && (!lif)) {
                   else if ((cp.x*np.y-np.x*cp.y)<0) {
                    lp.x += (np.x-cp.x);
                    lp.y += (np.y-cp.y);
                    
                    //lp.x = (np.x);
                    //lp.y = (np.y);
                    //float dist = sqrt(lp.x*lp.x + lp.y*lp.y);
                    //lp.d += (dist-cp.d);
                    //printf("\n 2222 neighbor delta index %ld %ld  coord %f %f lpd [%f  %f  =%f]\n",
                  //Neighbor->first.x- pngiter->first.x, Neighbor->first.y-pngiter->first.y,
                 // Neighbor->second.x - pngiter->second.x, Neighbor->second.y-pngiter->second.y, dist,cp.d, lp.d);
                    lp.count++;
                    lif = true;
                    lfind = true;
                  }
              }
              //if (lif && rif) break;
            } // 
          }
          if (lfind && rfind && (rp.count>=3)&&(lp.count>=3)) {
            rp.x = rp.x/rp.count;
            rp.y = rp.y/rp.count;

            lp.x = lp.x/lp.count;
            lp.y = lp.y/lp.count;


            float cosv = (rp.x*lp.x+rp.y*lp.y)/(sqrt(rp.x*rp.x+rp.y*rp.y)*sqrt(lp.x*lp.x+lp.y*lp.y));
            float theta = acos(cosv);
            float curvature = (lp.d+rp.d)*(lp.d+rp.d);
            if (lp.count > 1 && (fabs(lp.count-rp.count)<20)) {
              if (theta > M_PI*16/18)
              // if (curvature < 10.5)
              pngiter->second.lineType = 1;
              // else if (curvature > 50.0)
              else if (fabs(theta-M_PI*2/3)< M_PI/18)
              pngiter->second.lineType = 2;
            }
            

            ////////////////////////////////////////////////////////////////////
            if (pngiter->second.lineType == 1)  // ru guo dang qian dian pan ding wei zhixian, ze fu jin dian ye wei zhixian
            for (int l = -searchR5*2; l<searchR5*2; l++) {   // cong zui wai ceng kai shi cha zhao zuo you liang ce shuju 
              //for (float m = 0; m<=M_PI*2; m+=stheta) {
              for (float m = -searchR5*2; m<searchR5*2; m++) {
                //int64_t xinc = l*cos(m);
                //int64_t yinc = l*sin(m);
                int64_t xinc = l;
                int64_t yinc = m;
                //printf("search angle m= %f inc angle=%f  xyinc %ld  %ld\n",m,stheta, xinc, yinc);
                VOXEL_LOC pNeighbor(xcoord + xinc, ycoord +yinc, 0);
                auto Neighbor = png_map.find(pNeighbor);
                if (Neighbor != png_map.end() &&
                (Neighbor->second.vcnt >= zeroVcnt)) {

                  PNG_POINT cur_p;  //  
        
                  cur_p.x += (Neighbor->second.x-cp.x);
                  cur_p.y += (Neighbor->second.y-cp.y);
                  
                  float cosvbl = (cur_p.x*lp.x+cur_p.y*lp.y)/(sqrt(cur_p.x*cur_p.x+cur_p.y*cur_p.y)*sqrt(lp.x*lp.x+lp.y*lp.y));
                  float bl_theta = acos(cosvbl);
                  float cosvbr = (rp.x*cur_p.x+rp.y*cur_p.y)/(sqrt(rp.x*rp.x+rp.y*rp.y)*sqrt(cur_p.x*cur_p.x+cur_p.y*cur_p.y));
                  float br_theta = acos(cosvbl);

                  if ((bl_theta > M_PI*16/18) || (br_theta > M_PI*16/18))  
                  Neighbor->second.lineType = 1;
                }
                //if (lif && rif) break;
              } // 
            }

            ////////////////////////////////////////////////////////////////////



            //printf(" ceter [%f %f]  right %f :[%f %f ]  left %f :[%f %f ]  Theta=%f curvature=%f  %f %f %f\n", cp.x,cp.y,
            //rp.count, rp.x, rp.y, lp.count, lp.x, lp.y, theta,curvature, lp.d, rp.d,cp.d);
          }

      }

  }

  printf(" lsiz=%f total voxel number= %d  l1-l10[%d %d %d %d %d %d %d %d %d]\n", lsize, totalvoxels,
            l1,l2,l3,l4,l5,l6,l7,l8,l9);
    
    for (y = ysize; y >0 ; y--)
    for (x = 0; x < xsize; x++) {
      int64_t xcoord = x+xMin;
      int64_t ycoord = y+yMin;
      VOXEL_LOC position(xcoord, ycoord, 0);
      auto pngiter = png_map.find(position);
      bool modified = false;
      
      if (pngiter != png_map.end()) {
        if (pngiter->second.vcnt > minVcnt&&(pngiter->second.vcnt<13*lsize)) {  // must contain enough points
          float isolate = 0;
          float totalCnt = 0;
          float line_cnt = 0;
          for (int l = -searchRadius; l<=searchRadius; l++) {
            for (int m = -searchRadius; m<=searchRadius; m++) {
              VOXEL_LOC pNeighbor(xcoord + l, ycoord +m, 0);
              auto Neighbor = png_map.find(pNeighbor);
              if (Neighbor != png_map.end()) {
                  if (Neighbor->second.viewed) continue;
                  totalCnt += 1.0;
                  if (pngiter->second.vcnt < Neighbor->second.vcnt)
                    isolate = -65535;
                  else {
                    isolate += 1.0;
                    if (Neighbor->second.lineType==1) {
                      isolate -= 3.0;
                    }
                    
                  }

              }
            } // 
          }
          //printf("near point is isolate cnt =%f\n", isolate);
          if (isolate >= isolateRatio) {
            pngiter->second.isIsolate = true;
            for (int l = -searchRadius; l<=searchRadius; l++) {
            for (int m = -searchRadius; m<=searchRadius; m++) {
              VOXEL_LOC pNeighbor(xcoord + l, ycoord +m, 0);
              auto Neighbor = png_map.find(pNeighbor);
              if (Neighbor != png_map.end()) {
                   //Neighbor->second.viewed = true;
              }
            } // 
          }
          } 

        }
      }

  }

  std::unordered_map<VOXEL_LOC, PNG_POINT> png_final;
  unsigned char rgb[xsize * ysize * 3], *p = rgb;
  FILE *fp = fopen((resultPath+std::string("depthrgb.png")).c_str(), "wb");
  FILE *fponly = fopen((resultPath+std::string("Onlyrgb.png")).c_str(), "wb");
  for (y = ysize; y >0 ; y--)
    for (x = 0; x < xsize; x++) {
      int64_t xcoord = x+xMin;
      int64_t ycoord = y+yMin;
      VOXEL_LOC position(xcoord, ycoord, 0);
      auto pngiter = png_map.find(position);
      bool modified = false;
      
      if (pngiter != png_map.end()) {
        

        if (pngiter->second.vcnt > minVcnt) {  // must contain enough points
          bool isolate = pngiter->second.isIsolate;
          if (isolate)
          for (int l = -searchRadius; l<=searchRadius; l++) {
            for (int m = -searchRadius; m<=searchRadius; m++) {
              VOXEL_LOC pNeighbor(xcoord + l, ycoord +m, 0);
              auto Neighbor = png_map.find(pNeighbor);
              if (Neighbor != png_map.end()) {  // ru guo fu jin yi jing you gu li dian, ze bu zai jiang dang qian dian zhi wei gu li dian
                  //if (pngiter->second.vcnt < Neighbor->second.vcnt)
                  //isolate = false;
              }
            } // 
          }

          if (isolate && (!onlyGrid)) {
            pngiter->second.isIsolate = true;
            if (pngiter->second.lineType==2) {
              //pngiter->second.isIsolate = false;
              printf("set pole as corner feature\n");
              *p++ = 0;    /* R */
              *p++ = 255;    /* G */
              *p++ = 0;    /* B */
            } else {
              pngiter->second.lineType = 0;
              *p++ = 255;    /* R */
              *p++ = 0;    /* G */
              *p++ = 0;    /* B */
            }  
          } else {
            pngiter->second.isIsolate = isolate;
            if (pngiter->second.lineType==1 && (!onlyGrid)) {
              *p++ = 0;    /* R */
              *p++ = 255;    /* G */
              *p++ = 255;    /* B */
            } else if (pngiter->second.lineType==2 && (!onlyGrid)) {
              *p++ = 0;    /* R */
              *p++ = 0;    /* G */
              *p++ = 255;    /* B */
            } else {
            *p++ = 255 - 255.0* pngiter->second.vcnt / maxVcnt;    /* R */
            *p++ = 255 - 255.0* pngiter->second.vcnt / maxVcnt;    /* G */
            *p++ = 255 - 255.0* pngiter->second.vcnt / maxVcnt;    /* B */
            }

          }
          PNG_POINT anp;
          png_final[position] = png_map[position];
          modified = true;
        }
      }
      if (!modified) {
        *p++ = 255;    /* R */
        *p++ = 255;    /* G */
        *p++ = 255;    /* B */
      }
  }

  svpng(fp, xsize, ysize, rgb, 0);
  fclose(fp);
  if (onlyGrid)
  svpng(fponly, xsize, ysize, rgb, 0);
  fclose(fponly);
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(
              new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(
              new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr corner_cloud(
              new pcl::PointCloud<pcl::PointXYZI>());
  plsize = points_sv.size();
  for (uint i = 0; i < plsize; i++) {
    pcl::PointXYZI &p_c = points_sv[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], 0);

    auto pngiter = png_map.find(position);
      if (pngiter != png_map.end()) {
        int h = pngiter->second.vcnt;
        totalvoxels++;
        if (h>0 && h<=1 && pngiter->second.belongGround) {
          ground_cloud->push_back(p_c);
        }
        
      }

    auto iter = png_final.find(position);
    if (iter != png_final.end()&&(iter->second.lineType!=1)) {
      
      p_c.intensity = 0;
      if (iter->second.lineType == 2){
        p_c.intensity = 254;
        corner_cloud->push_back(p_c);
      }

      else if (iter->second.isIsolate){
          p_c.intensity = 125;
          current_cloud->push_back(p_c);
      }

    }
  }

  pcl::io::savePCDFileBinary((resultPath+std::string("tree_filtered.pcd")).c_str(), *current_cloud);   
  pcl::io::savePCDFileBinary((resultPath+std::string("ground.pcd")).c_str(), *ground_cloud);
  pcl::io::savePCDFileBinary((resultPath+std::string("corner.pcd")).c_str(), *corner_cloud);
  //pcl::io::savePCDFileBinary("/home/shinva/mappingData/js_filtered.pcd", *current_cloud);
  //pcl::io::savePCDFileBinary("/home/shinva/mappingData/js_ground.pcd", *ground_cloud);
  //pcl::io::savePCDFileBinary("/home/shinva/mappingData/js_corner.pcd", *corner_cloud);      
  //for (auto iter = png_map.begin(); iter != png_map.end(); ++iter) {
  //  std::cout << "--voxel map-"
  //          << "x: " << iter->first.x << "  y: " << iter->first.y  << std::endl;
  //}
  std::cout << "----------------voxel map max main-------------------"
            << "maxVcnt: " << maxVcnt << std::endl;
  std::cout << "xmax: " << xMax <<" xmin: " << xMin << " ymax: " << yMax <<" ymin: " << yMin
            << "zmax: " << zMax <<" zmin: " << zMin << std::endl;
}

void read_parameters(ros::NodeHandle &nh, ConfigSetting &config_setting) {

  // pre-preocess
  nh.param<double>("ds_size", config_setting.ds_size_, 0.5);
  nh.param<int>("maximum_corner_num", config_setting.maximum_corner_num_, 100);

  // key points
  nh.param<double>("plane_merge_normal_thre",
                   config_setting.plane_merge_normal_thre_, 0.1);
  nh.param<double>("plane_detection_thre", config_setting.plane_detection_thre_,
                   0.01);
  nh.param<double>("voxel_size", config_setting.voxel_size_, 2.0);
  nh.param<int>("voxel_init_num", config_setting.voxel_init_num_, 10);
  nh.param<double>("proj_image_resolution",
                   config_setting.proj_image_resolution_, 0.5);
  nh.param<double>("proj_dis_min", config_setting.proj_dis_min_, 0);
  nh.param<double>("proj_dis_max", config_setting.proj_dis_max_, 2);
  nh.param<double>("corner_thre", config_setting.corner_thre_, 10);

  // std descriptor
  nh.param<int>("effective_count", config_setting.effective_count, 2);
  nh.param<int>("pole_count", config_setting.pole_count, 5);
  nh.param<int>("surround_count", config_setting.surround_count,
                   4);
  nh.param<int>("stand_out_count",
                   config_setting.stand_out_count, 15);
  nh.param<int>("search_radius", config_setting.search_radius,
                   3);

  // candidate search
  nh.param<int>("skip_near_num", config_setting.skip_near_num_, 50);
  nh.param<int>("candidate_num", config_setting.candidate_num_, 50);
  nh.param<int>("sub_frame_num", config_setting.sub_frame_num_, 10);
  nh.param<double>("rough_dis_threshold", config_setting.rough_dis_threshold_,
                   0.01);
  nh.param<double>("vertex_diff_threshold",
                   config_setting.vertex_diff_threshold_, 0.5);
  nh.param<double>("icp_threshold", config_setting.icp_threshold_, 0.5);
  nh.param<double>("normal_threshold", config_setting.normal_threshold_, 0.2);
  nh.param<double>("dis_threshold", config_setting.dis_threshold_, 0.5);

  std::cout << "Sucessfully load parameters:" << std::endl;
  std::cout << "----------------Main Parameters-------------------"
            << std::endl;
  std::cout << "voxel size:" << config_setting.voxel_size_ << std::endl;
  std::cout << "loop detection threshold: " << config_setting.icp_threshold_
            << std::endl;
  std::cout << "sub-frame number: " << config_setting.sub_frame_num_
            << std::endl;
  std::cout << "candidate number: " << config_setting.candidate_num_
            << std::endl;
  std::cout << "maximum corners size: " << config_setting.maximum_corner_num_
            << std::endl;
}

void load_pose_with_time(
    const std::string &pose_file,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &poses_vec,
    std::vector<double> &times_vec) {
  times_vec.clear();
  poses_vec.clear();
  std::ifstream fin(pose_file);
  std::string line;
  Eigen::Matrix<double, 1, 7> temp_matrix;
  while (getline(fin, line)) {
    std::istringstream sin(line);
    std::vector<std::string> Waypoints;
    std::string info;
    int number = 0;
    while (getline(sin, info, ' ')) {
      if (number == 0) {
        double time;
        std::stringstream data;
        data << info;
        data >> time;
        times_vec.push_back(time);
        number++;
      } else {
        double p;
        std::stringstream data;
        data << info;
        data >> p;
        temp_matrix[number - 1] = p;
        if (number == 7) {
          Eigen::Vector3d translation(temp_matrix[0], temp_matrix[1],
                                      temp_matrix[2]);
          Eigen::Quaterniond q(temp_matrix[6], temp_matrix[3], temp_matrix[4],
                               temp_matrix[5]);
          std::pair<Eigen::Vector3d, Eigen::Matrix3d> single_pose;
          single_pose.first = translation;
          single_pose.second = q.toRotationMatrix();
          poses_vec.push_back(single_pose);
        }
        number++;
      }
    }
  }
}

double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(t_end -
                                                                   t_begin)
             .count() *
         1000;
}

pcl::PointXYZI vec2point(const Eigen::Vector3d &vec) {
  pcl::PointXYZI pi;
  pi.x = vec[0];
  pi.y = vec[1];
  pi.z = vec[2];
  return pi;
}
Eigen::Vector3d point2vec(const pcl::PointXYZI &pi) {
  return Eigen::Vector3d(pi.x, pi.y, pi.z);
}

bool attach_greater_sort(std::pair<double, int> a, std::pair<double, int> b) {
  return (a.first > b.first);
}

void publish_std_pairs(
    const std::vector<std::pair<STDesc, STDesc>> &match_std_pairs,
    const ros::Publisher &std_publisher) {
  visualization_msgs::MarkerArray ma_line;
  visualization_msgs::Marker m_line;
  m_line.type = visualization_msgs::Marker::LINE_LIST;
  m_line.action = visualization_msgs::Marker::ADD;
  m_line.ns = "lines";
  // Don't forget to set the alpha!
  m_line.scale.x = 0.25;
  m_line.pose.orientation.w = 1.0;
  m_line.header.frame_id = "camera_init";
  m_line.id = 0;
  int max_pub_cnt = 1;
  for (auto var : match_std_pairs) {
    if (max_pub_cnt > 100) {
      break;
    }
    max_pub_cnt++;
    m_line.color.a = 0.8;
    m_line.points.clear();
    m_line.color.r = 138.0 / 255;
    m_line.color.g = 226.0 / 255;
    m_line.color.b = 52.0 / 255;
    geometry_msgs::Point p;
    p.x = var.second.vertex_A_[0];
    p.y = var.second.vertex_A_[1];
    p.z = var.second.vertex_A_[2];
    Eigen::Vector3d t_p;
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    p.x = var.second.vertex_B_[0];
    p.y = var.second.vertex_B_[1];
    p.z = var.second.vertex_B_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.second.vertex_C_[0];
    p.y = var.second.vertex_C_[1];
    p.z = var.second.vertex_C_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    p.x = var.second.vertex_B_[0];
    p.y = var.second.vertex_B_[1];
    p.z = var.second.vertex_B_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.second.vertex_C_[0];
    p.y = var.second.vertex_C_[1];
    p.z = var.second.vertex_C_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    p.x = var.second.vertex_A_[0];
    p.y = var.second.vertex_A_[1];
    p.z = var.second.vertex_A_[2];
    t_p << p.x, p.y, p.z;
    p.x = t_p[0];
    p.y = t_p[1];
    p.z = t_p[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    // another
    m_line.points.clear();
    m_line.color.r = 1;
    m_line.color.g = 1;
    m_line.color.b = 1;
    p.x = var.first.vertex_A_[0];
    p.y = var.first.vertex_A_[1];
    p.z = var.first.vertex_A_[2];
    m_line.points.push_back(p);
    p.x = var.first.vertex_B_[0];
    p.y = var.first.vertex_B_[1];
    p.z = var.first.vertex_B_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.first.vertex_C_[0];
    p.y = var.first.vertex_C_[1];
    p.z = var.first.vertex_C_[2];
    m_line.points.push_back(p);
    p.x = var.first.vertex_B_[0];
    p.y = var.first.vertex_B_[1];
    p.z = var.first.vertex_B_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
    p.x = var.first.vertex_C_[0];
    p.y = var.first.vertex_C_[1];
    p.z = var.first.vertex_C_[2];
    m_line.points.push_back(p);
    p.x = var.first.vertex_A_[0];
    p.y = var.first.vertex_A_[1];
    p.z = var.first.vertex_A_[2];
    m_line.points.push_back(p);
    ma_line.markers.push_back(m_line);
    m_line.id++;
    m_line.points.clear();
  }
  for (int j = 0; j < 100 * 6; j++) {
    m_line.color.a = 0.00;
    ma_line.markers.push_back(m_line);
    m_line.id++;
  }
  std_publisher.publish(ma_line);
  m_line.id = 0;
  ma_line.markers.clear();
}


void STDescManager::SearchLoop(
    const std::vector<STDesc> &stds_vec, std::pair<int, double> &loop_result,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
    std::vector<std::pair<STDesc, STDesc>> &loop_std_pair) {

  if (stds_vec.size() == 0) {
    ROS_ERROR_STREAM("No STDescs!");
    loop_result = std::pair<int, double>(-1, 0);
    return;
  }
  // step1, select candidates, default number 50
  auto t1 = std::chrono::high_resolution_clock::now();
  std::vector<STDMatchList> candidate_matcher_vec;
  candidate_selector(stds_vec, candidate_matcher_vec);

  auto t2 = std::chrono::high_resolution_clock::now();
  // step2, select best candidates from rough candidates
  double best_score = 0;
  unsigned int best_candidate_id = -1;
  unsigned int triggle_candidate = -1;
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> best_transform;
  std::vector<std::pair<STDesc, STDesc>> best_sucess_match_vec;
  for (size_t i = 0; i < candidate_matcher_vec.size(); i++) {
    double verify_score = -1;
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> relative_pose;
    std::vector<std::pair<STDesc, STDesc>> sucess_match_vec;
    candidate_verify(candidate_matcher_vec[i], verify_score, relative_pose,
                     sucess_match_vec);
    if (verify_score > best_score) {
      best_score = verify_score;
      best_candidate_id = candidate_matcher_vec[i].match_id_.second;
      best_transform = relative_pose;
      best_sucess_match_vec = sucess_match_vec;
      triggle_candidate = i;
    }
  }
  auto t3 = std::chrono::high_resolution_clock::now();

  // std::cout << "[Time] candidate selector: " << time_inc(t2, t1)
  //           << " ms, candidate verify: " << time_inc(t3, t2) << "ms"
  //           << std::endl;

  if (best_score > config_setting_.icp_threshold_) {
    loop_result = std::pair<int, double>(best_candidate_id, best_score);
    loop_transform = best_transform;
    loop_std_pair = best_sucess_match_vec;
    return;
  } else {
    loop_result = std::pair<int, double>(-1, 0);
    return;
  }
}

void STDescManager::AddSTDescs(const std::vector<STDesc> &stds_vec) {
  // update frame id
  current_frame_id_++;
  for (auto single_std : stds_vec) {
    // calculate the position of single std
    STDesc_LOC position;
    position.x = (int)(single_std.side_length_[0] + 0.5);
    position.y = (int)(single_std.side_length_[1] + 0.5);
    position.z = (int)(single_std.side_length_[2] + 0.5);
    position.a = (int)(single_std.angle_[0]);
    position.b = (int)(single_std.angle_[1]);
    position.c = (int)(single_std.angle_[2]);
    auto iter = data_base_.find(position);
    if (iter != data_base_.end()) {
      data_base_[position].push_back(single_std);
    } else {
      std::vector<STDesc> descriptor_vec;
      descriptor_vec.push_back(single_std);
      data_base_[position] = descriptor_vec;
    }
  }
  return;
}

void STDescManager::init_voxel_map(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map) {
  uint plsize = input_cloud->size();
  for (uint i = 0; i < plsize; i++) {
    Eigen::Vector3d p_c(input_cloud->points[i].x, input_cloud->points[i].y,
                        input_cloud->points[i].z);
    double loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c[j] / config_setting_.voxel_size_;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = voxel_map.find(position);
    if (iter != voxel_map.end()) {
      voxel_map[position]->voxel_points_.push_back(p_c);
    } else {
      OctoTree *octo_tree = new OctoTree(config_setting_);
      voxel_map[position] = octo_tree;
      voxel_map[position]->voxel_points_.push_back(p_c);
    }
  }
  std::vector<std::unordered_map<VOXEL_LOC, OctoTree *>::iterator> iter_list;
  std::vector<size_t> index;
  size_t i = 0;
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) {
    index.push_back(i);
    i++;
    iter_list.push_back(iter);
  }
  // speed up initialization
  // #ifdef MP_EN
  //   omp_set_num_threads(MP_PROC_NUM);
  //   std::cout << "omp num:" << MP_PROC_NUM << std::endl;
  // #pragma omp parallel for
  // #endif
  for (int i = 0; i < index.size(); i++) {
    iter_list[i]->second->init_octo_tree();
  }
  // std::cout << "voxel num:" << index.size() << std::endl;
  // std::for_each(
  //     std::execution::par_unseq, index.begin(), index.end(),
  //     [&](const size_t &i) { iter_list[i]->second->init_octo_tree(); });
}

void STDescManager::build_connection(
    std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map) {
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    if (iter->second->plane_ptr_->is_plane_) {
      OctoTree *current_octo = iter->second;
      for (int i = 0; i < 6; i++) {
        VOXEL_LOC neighbor = iter->first;
        if (i == 0) {
          neighbor.x = neighbor.x + 1;
        } else if (i == 1) {
          neighbor.y = neighbor.y + 1;
        } else if (i == 2) {
          neighbor.z = neighbor.z + 1;
        } else if (i == 3) {
          neighbor.x = neighbor.x - 1;
        } else if (i == 4) {
          neighbor.y = neighbor.y - 1;
        } else if (i == 5) {
          neighbor.z = neighbor.z - 1;
        }
        auto near = voxel_map.find(neighbor);
        if (near == voxel_map.end()) {
          current_octo->is_check_connect_[i] = true;
          current_octo->connect_[i] = false;
        } else {
          if (!current_octo->is_check_connect_[i]) {
            OctoTree *near_octo = near->second;
            current_octo->is_check_connect_[i] = true;
            int j;
            if (i >= 3) {
              j = i - 3;
            } else {
              j = i + 3;
            }
            near_octo->is_check_connect_[j] = true;
            if (near_octo->plane_ptr_->is_plane_) {
              // merge near octo
              Eigen::Vector3d normal_diff = current_octo->plane_ptr_->normal_ -
                                            near_octo->plane_ptr_->normal_;
              Eigen::Vector3d normal_add = current_octo->plane_ptr_->normal_ +
                                           near_octo->plane_ptr_->normal_;
              if (normal_diff.norm() <
                      config_setting_.plane_merge_normal_thre_ ||
                  normal_add.norm() <
                      config_setting_.plane_merge_normal_thre_) {
                current_octo->connect_[i] = true;
                near_octo->connect_[j] = true;
                current_octo->connect_tree_[i] = near_octo;
                near_octo->connect_tree_[j] = current_octo;
              } else {
                current_octo->connect_[i] = false;
                near_octo->connect_[j] = false;
              }
            } else {
              current_octo->connect_[i] = false;
              near_octo->connect_[j] = true;
              near_octo->connect_tree_[j] = current_octo;
            }
          }
        }
      }
    }
  }
}

void STDescManager::getPlane(
    const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud) {
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    if (iter->second->plane_ptr_->is_plane_) {
      pcl::PointXYZINormal pi;
      pi.x = iter->second->plane_ptr_->center_[0];
      pi.y = iter->second->plane_ptr_->center_[1];
      pi.z = iter->second->plane_ptr_->center_[2];
      pi.normal_x = iter->second->plane_ptr_->normal_[0];
      pi.normal_y = iter->second->plane_ptr_->normal_[1];
      pi.normal_z = iter->second->plane_ptr_->normal_[2];
      plane_cloud->push_back(pi);
    }
  }
}

void STDescManager::corner_extractor(
    std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points) {

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr prepare_corner_points(
      new pcl::PointCloud<pcl::PointXYZINormal>);

  // Avoid inconsistent voxel cutting caused by different view point
  std::vector<Eigen::Vector3i> voxel_round;
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      for (int z = -1; z <= 1; z++) {
        Eigen::Vector3i voxel_inc(x, y, z);
        voxel_round.push_back(voxel_inc);
      }
    }
  }
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    if (!iter->second->plane_ptr_->is_plane_) {
      VOXEL_LOC current_position = iter->first;
      OctoTree *current_octo = iter->second;
      int connect_index = -1;
      for (int i = 0; i < 6; i++) {
        if (current_octo->connect_[i]) {
          connect_index = i;
          OctoTree *connect_octo = current_octo->connect_tree_[connect_index];
          bool use = false;
          for (int j = 0; j < 6; j++) {
            if (connect_octo->is_check_connect_[j]) {
              if (connect_octo->connect_[j]) {
                use = true;
              }
            }
          }
          // if no plane near the voxel, skip
          if (use == false) {
            continue;
          }
          // only project voxels with points num > 10
          if (current_octo->voxel_points_.size() > 10) {
            Eigen::Vector3d projection_normal =
                current_octo->connect_tree_[connect_index]->plane_ptr_->normal_;
            Eigen::Vector3d projection_center =
                current_octo->connect_tree_[connect_index]->plane_ptr_->center_;
            std::vector<Eigen::Vector3d> proj_points;
            // proj the boundary voxel and nearby voxel onto adjacent plane
            for (auto voxel_inc : voxel_round) {
              VOXEL_LOC connect_project_position = current_position;
              connect_project_position.x += voxel_inc[0];
              connect_project_position.y += voxel_inc[1];
              connect_project_position.z += voxel_inc[2];
              auto iter_near = voxel_map.find(connect_project_position);
              if (iter_near != voxel_map.end()) {
                bool skip_flag = false;
                if (!voxel_map[connect_project_position]
                         ->plane_ptr_->is_plane_) {
                  if (voxel_map[connect_project_position]->is_project_) {
                    for (auto normal : voxel_map[connect_project_position]
                                           ->proj_normal_vec_) {
                      Eigen::Vector3d normal_diff = projection_normal - normal;
                      Eigen::Vector3d normal_add = projection_normal + normal;
                      // check if repeated project
                      if (normal_diff.norm() < 0.5 || normal_add.norm() < 0.5) {
                        skip_flag = true;
                      }
                    }
                  }
                  if (skip_flag) {
                    continue;
                  }
                  for (size_t j = 0; j < voxel_map[connect_project_position]
                                             ->voxel_points_.size();
                       j++) {
                    proj_points.push_back(
                        voxel_map[connect_project_position]->voxel_points_[j]);
                    voxel_map[connect_project_position]->is_project_ = true;
                    voxel_map[connect_project_position]
                        ->proj_normal_vec_.push_back(projection_normal);
                  }
                }
              }
            }
            // here do the 2D projection and corner extraction
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr sub_corner_points(
                new pcl::PointCloud<pcl::PointXYZINormal>);
            extract_corner(projection_center, projection_normal, proj_points,
                           sub_corner_points);
            for (auto pi : sub_corner_points->points) {
              prepare_corner_points->push_back(pi);
            }
          }
        }
      }
    }
  }
  // non_maxi_suppression(prepare_corner_points);

  if (config_setting_.maximum_corner_num_ > prepare_corner_points->size()) {
    corner_points = prepare_corner_points;
  } else {
    std::vector<std::pair<double, int>> attach_vec;
    for (size_t i = 0; i < prepare_corner_points->size(); i++) {
      attach_vec.push_back(std::pair<double, int>(
          prepare_corner_points->points[i].intensity, i));
    }
    std::sort(attach_vec.begin(), attach_vec.end(), attach_greater_sort);
    for (size_t i = 0; i < config_setting_.maximum_corner_num_; i++) {
      corner_points->points.push_back(
          prepare_corner_points->points[attach_vec[i].second]);
    }
  }
}

void STDescManager::extract_corner(
    const Eigen::Vector3d &proj_center, const Eigen::Vector3d proj_normal,
    const std::vector<Eigen::Vector3d> proj_points,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points) {

  double resolution = config_setting_.proj_image_resolution_;
  double dis_threshold_min = config_setting_.proj_dis_min_;
  double dis_threshold_max = config_setting_.proj_dis_max_;
  double A = proj_normal[0];
  double B = proj_normal[1];
  double C = proj_normal[2];
  double D = -(A * proj_center[0] + B * proj_center[1] + C * proj_center[2]);
  Eigen::Vector3d x_axis(1, 1, 0);
  if (C != 0) {
    x_axis[2] = -(A + B) / C;
  } else if (B != 0) {
    x_axis[1] = -A / B;
  } else {
    x_axis[0] = 0;
    x_axis[1] = 1;
  }
  x_axis.normalize();
  Eigen::Vector3d y_axis = proj_normal.cross(x_axis);
  y_axis.normalize();
  double ax = x_axis[0];
  double bx = x_axis[1];
  double cx = x_axis[2];
  double dx =
      -(ax * proj_center[0] + bx * proj_center[1] + cx * proj_center[2]);
  double ay = y_axis[0];
  double by = y_axis[1];
  double cy = y_axis[2];
  double dy =
      -(ay * proj_center[0] + by * proj_center[1] + cy * proj_center[2]);
  std::vector<Eigen::Vector2d> point_list_2d;
  for (size_t i = 0; i < proj_points.size(); i++) {
    double x = proj_points[i][0];
    double y = proj_points[i][1];
    double z = proj_points[i][2];
    double dis = fabs(x * A + y * B + z * C + D);
    if (dis < dis_threshold_min || dis > dis_threshold_max) {
      continue;
    }
    Eigen::Vector3d cur_project;

    cur_project[0] = (-A * (B * y + C * z + D) + x * (B * B + C * C)) /
                     (A * A + B * B + C * C);
    cur_project[1] = (-B * (A * x + C * z + D) + y * (A * A + C * C)) /
                     (A * A + B * B + C * C);
    cur_project[2] = (-C * (A * x + B * y + D) + z * (A * A + B * B)) /
                     (A * A + B * B + C * C);
    pcl::PointXYZ p;
    p.x = cur_project[0];
    p.y = cur_project[1];
    p.z = cur_project[2];
    double project_x =
        cur_project[0] * ay + cur_project[1] * by + cur_project[2] * cy + dy;
    double project_y =
        cur_project[0] * ax + cur_project[1] * bx + cur_project[2] * cx + dx;
    Eigen::Vector2d p_2d(project_x, project_y);
    point_list_2d.push_back(p_2d);
  }
  double min_x = 10;
  double max_x = -10;
  double min_y = 10;
  double max_y = -10;
  if (point_list_2d.size() <= 5) {
    return;
  }
  for (auto pi : point_list_2d) {
    if (pi[0] < min_x) {
      min_x = pi[0];
    }
    if (pi[0] > max_x) {
      max_x = pi[0];
    }
    if (pi[1] < min_y) {
      min_y = pi[1];
    }
    if (pi[1] > max_y) {
      max_y = pi[1];
    }
  }
  // segment project cloud with a fixed resolution
  int segmen_base_num = 5;
  double segmen_len = segmen_base_num * resolution;
  int x_segment_num = (max_x - min_x) / segmen_len + 1;
  int y_segment_num = (max_y - min_y) / segmen_len + 1;
  int x_axis_len = (int)((max_x - min_x) / resolution + segmen_base_num);
  int y_axis_len = (int)((max_y - min_y) / resolution + segmen_base_num);
  std::vector<Eigen::Vector2d> img_container[x_axis_len][y_axis_len];
  double img_count_array[x_axis_len][y_axis_len] = {0};
  double gradient_array[x_axis_len][y_axis_len] = {0};
  double mean_x_array[x_axis_len][y_axis_len] = {0};
  double mean_y_array[x_axis_len][y_axis_len] = {0};
  for (int x = 0; x < x_axis_len; x++) {
    for (int y = 0; y < y_axis_len; y++) {
      img_count_array[x][y] = 0;
      mean_x_array[x][y] = 0;
      mean_y_array[x][y] = 0;
      gradient_array[x][y] = 0;
      std::vector<Eigen::Vector2d> single_container;
      img_container[x][y] = single_container;
    }
  }
  for (size_t i = 0; i < point_list_2d.size(); i++) {
    int x_index = (int)((point_list_2d[i][0] - min_x) / resolution);
    int y_index = (int)((point_list_2d[i][1] - min_y) / resolution);
    mean_x_array[x_index][y_index] += point_list_2d[i][0];
    mean_y_array[x_index][y_index] += point_list_2d[i][1];
    img_count_array[x_index][y_index]++;
    img_container[x_index][y_index].push_back(point_list_2d[i]);
  }
  // calc gradient
  for (int x = 0; x < x_axis_len; x++) {
    for (int y = 0; y < y_axis_len; y++) {
      double gradient = 0;
      int cnt = 0;
      int inc = 1;
      for (int x_inc = -inc; x_inc <= inc; x_inc++) {
        for (int y_inc = -inc; y_inc <= inc; y_inc++) {
          int xx = x + x_inc;
          int yy = y + y_inc;
          if (xx >= 0 && xx < x_axis_len && yy >= 0 && yy < y_axis_len) {
            if (xx != x || yy != y) {
              if (img_count_array[xx][yy] >= 0) {
                gradient += img_count_array[x][y] - img_count_array[xx][yy];
                cnt++;
              }
            }
          }
        }
      }
      if (cnt != 0) {
        gradient_array[x][y] = gradient * 1.0 / cnt;
      } else {
        gradient_array[x][y] = 0;
      }
    }
  }
  // extract corner by gradient
  std::vector<int> max_gradient_vec;
  std::vector<int> max_gradient_x_index_vec;
  std::vector<int> max_gradient_y_index_vec;
  for (int x_segment_index = 0; x_segment_index < x_segment_num;
       x_segment_index++) {
    for (int y_segment_index = 0; y_segment_index < y_segment_num;
         y_segment_index++) {
      double max_gradient = 0;
      int max_gradient_x_index = -10;
      int max_gradient_y_index = -10;
      for (int x_index = x_segment_index * segmen_base_num;
           x_index < (x_segment_index + 1) * segmen_base_num; x_index++) {
        for (int y_index = y_segment_index * segmen_base_num;
             y_index < (y_segment_index + 1) * segmen_base_num; y_index++) {
          if (img_count_array[x_index][y_index] > max_gradient) {
            max_gradient = img_count_array[x_index][y_index];
            max_gradient_x_index = x_index;
            max_gradient_y_index = y_index;
          }
        }
      }
      if (max_gradient >= config_setting_.corner_thre_) {
        max_gradient_vec.push_back(max_gradient);
        max_gradient_x_index_vec.push_back(max_gradient_x_index);
        max_gradient_y_index_vec.push_back(max_gradient_y_index);
      }
    }
  }
  // filter out line
  // calc line or not
  std::vector<Eigen::Vector2i> direction_list;
  Eigen::Vector2i d(0, 1);
  direction_list.push_back(d);
  d << 1, 0;
  direction_list.push_back(d);
  d << 1, 1;
  direction_list.push_back(d);
  d << 1, -1;
  direction_list.push_back(d);
  for (size_t i = 0; i < max_gradient_vec.size(); i++) {
    bool is_add = true;
    for (int j = 0; j < 4; j++) {
      Eigen::Vector2i p(max_gradient_x_index_vec[i],
                        max_gradient_y_index_vec[i]);
      Eigen::Vector2i p1 = p + direction_list[j];
      Eigen::Vector2i p2 = p - direction_list[j];
      int threshold = img_count_array[p[0]][p[1]] / 2;
      if (img_count_array[p1[0]][p1[1]] >= threshold &&
          img_count_array[p2[0]][p2[1]] >= threshold) {
        // is_add = false;
      } else {
        continue;
      }
    }
    if (is_add) {
      double px = mean_x_array[max_gradient_x_index_vec[i]]
                              [max_gradient_y_index_vec[i]] /
                  img_count_array[max_gradient_x_index_vec[i]]
                                 [max_gradient_y_index_vec[i]];
      double py = mean_y_array[max_gradient_x_index_vec[i]]
                              [max_gradient_y_index_vec[i]] /
                  img_count_array[max_gradient_x_index_vec[i]]
                                 [max_gradient_y_index_vec[i]];
      // reproject on 3D space
      Eigen::Vector3d coord = py * x_axis + px * y_axis + proj_center;
      std::cout << py <<"x_axis" <<x_axis <<std::endl;
      std::cout << px <<"y_axis" <<y_axis <<std::endl;
      std::cout << px <<"proj_normal" <<proj_normal <<std::endl;
      pcl::PointXYZINormal pi;
      pi.x = coord[0];
      pi.y = coord[1];
      pi.z = coord[2];
      pi.intensity = max_gradient_vec[i];
      pi.normal_x = proj_normal[0];
      pi.normal_y = proj_normal[1];
      pi.normal_z = proj_normal[2];
      corner_points->points.push_back(pi);
    }
  }
  return;
}




void STDescManager::candidate_selector(
    const std::vector<STDesc> &stds_vec,
    std::vector<STDMatchList> &candidate_matcher_vec) {
  double match_array[MAX_FRAME_N] = {0};
  std::vector<std::pair<STDesc, STDesc>> match_vec;
  std::vector<int> match_index_vec;
  std::vector<Eigen::Vector3i> voxel_round;
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      for (int z = -1; z <= 1; z++) {
        Eigen::Vector3i voxel_inc(x, y, z);
        voxel_round.push_back(voxel_inc);
      }
    }
  }

  std::vector<bool> useful_match(stds_vec.size());
  std::vector<std::vector<size_t>> useful_match_index(stds_vec.size());
  std::vector<std::vector<STDesc_LOC>> useful_match_position(stds_vec.size());
  std::vector<size_t> index(stds_vec.size());
  for (size_t i = 0; i < index.size(); ++i) {
    index[i] = i;
    useful_match[i] = false;
  }
  // speed up matching
  int dis_match_cnt = 0;
  int final_match_cnt = 0;
#ifdef MP_EN
  // omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
  for (size_t i = 0; i < stds_vec.size(); i++) {
    STDesc src_std = stds_vec[i];
    STDesc_LOC position;
    int best_index = 0;
    STDesc_LOC best_position;
    double dis_threshold =
        src_std.side_length_.norm() * config_setting_.rough_dis_threshold_;
    for (auto voxel_inc : voxel_round) {
      position.x = (int)(src_std.side_length_[0] + voxel_inc[0]);
      position.y = (int)(src_std.side_length_[1] + voxel_inc[1]);
      position.z = (int)(src_std.side_length_[2] + voxel_inc[2]);
      Eigen::Vector3d voxel_center((double)position.x + 0.5,
                                   (double)position.y + 0.5,
                                   (double)position.z + 0.5);
      if ((src_std.side_length_ - voxel_center).norm() < 1.5) {
        auto iter = data_base_.find(position);
        if (iter != data_base_.end()) {
          for (size_t j = 0; j < data_base_[position].size(); j++) {
            if ((src_std.frame_id_ - data_base_[position][j].frame_id_) >
                config_setting_.skip_near_num_) {
              double dis =
                  (src_std.side_length_ - data_base_[position][j].side_length_)
                      .norm();
              // rough filter with side lengths
              if (dis < dis_threshold) {
                dis_match_cnt++;
                // rough filter with vertex attached info
                double vertex_attach_diff =
                    2.0 *
                    (src_std.vertex_attached_ -
                     data_base_[position][j].vertex_attached_)
                        .norm() /
                    (src_std.vertex_attached_ +
                     data_base_[position][j].vertex_attached_)
                        .norm();
                // std::cout << "vertex diff:" << vertex_attach_diff <<
                // std::endl;
                if (vertex_attach_diff <
                    config_setting_.vertex_diff_threshold_) {
                  final_match_cnt++;
                  useful_match[i] = true;
                  useful_match_position[i].push_back(position);
                  useful_match_index[i].push_back(j);
                }
              }
            }
          }
        }
      }
    }
  }
  // std::cout << "dis match num:" << dis_match_cnt
  //           << ", final match num:" << final_match_cnt << std::endl;

  // record match index
  std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>>
      index_recorder;
  for (size_t i = 0; i < useful_match.size(); i++) {
    if (useful_match[i]) {
      for (size_t j = 0; j < useful_match_index[i].size(); j++) {
        match_array[data_base_[useful_match_position[i][j]]
                              [useful_match_index[i][j]]
                                  .frame_id_] += 1;
        Eigen::Vector2i match_index(i, j);
        index_recorder.push_back(match_index);
        match_index_vec.push_back(
            data_base_[useful_match_position[i][j]][useful_match_index[i][j]]
                .frame_id_);
      }
    }
  }

  // select candidate according to the matching score
  for (int cnt = 0; cnt < config_setting_.candidate_num_; cnt++) {
    double max_vote = 1;
    int max_vote_index = -1;
    for (int i = 0; i < MAX_FRAME_N; i++) {
      if (match_array[i] > max_vote) {
        max_vote = match_array[i];
        max_vote_index = i;
      }
    }
    STDMatchList match_triangle_list;
    if (max_vote_index >= 0 && max_vote >= 5) {
      match_array[max_vote_index] = 0;
      match_triangle_list.match_id_.first = current_frame_id_;
      match_triangle_list.match_id_.second = max_vote_index;
      for (size_t i = 0; i < index_recorder.size(); i++) {
        if (match_index_vec[i] == max_vote_index) {
          std::pair<STDesc, STDesc> single_match_pair;
          single_match_pair.first = stds_vec[index_recorder[i][0]];
          single_match_pair.second =
              data_base_[useful_match_position[index_recorder[i][0]]
                                              [index_recorder[i][1]]]
                        [useful_match_index[index_recorder[i][0]]
                                           [index_recorder[i][1]]];
          match_triangle_list.match_list_.push_back(single_match_pair);
        }
      }
      candidate_matcher_vec.push_back(match_triangle_list);
    } else {
      break;
    }
  }
}

// Get the best candidate frame by geometry check
void STDescManager::candidate_verify(
    const STDMatchList &candidate_matcher, double &verify_score,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
    std::vector<std::pair<STDesc, STDesc>> &sucess_match_vec) {
  sucess_match_vec.clear();
  int skip_len = (int)(candidate_matcher.match_list_.size() / 50) + 1;
  int use_size = candidate_matcher.match_list_.size() / skip_len;
  double dis_threshold = 3.0;
  std::vector<size_t> index(use_size);
  std::vector<int> vote_list(use_size);
  for (size_t i = 0; i < index.size(); i++) {
    index[i] = i;
  }
  std::mutex mylock;

#ifdef MP_EN
  // omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
  for (size_t i = 0; i < use_size; i++) {
    auto single_pair = candidate_matcher.match_list_[i * skip_len];
    int vote = 0;
    Eigen::Matrix3d test_rot;
    Eigen::Vector3d test_t;
    triangle_solver(single_pair, test_t, test_rot);
    for (size_t j = 0; j < candidate_matcher.match_list_.size(); j++) {
      auto verify_pair = candidate_matcher.match_list_[j];
      Eigen::Vector3d A = verify_pair.first.vertex_A_;
      Eigen::Vector3d A_transform = test_rot * A + test_t;
      Eigen::Vector3d B = verify_pair.first.vertex_B_;
      Eigen::Vector3d B_transform = test_rot * B + test_t;
      Eigen::Vector3d C = verify_pair.first.vertex_C_;
      Eigen::Vector3d C_transform = test_rot * C + test_t;
      double dis_A = (A_transform - verify_pair.second.vertex_A_).norm();
      double dis_B = (B_transform - verify_pair.second.vertex_B_).norm();
      double dis_C = (C_transform - verify_pair.second.vertex_C_).norm();
      if (dis_A < dis_threshold && dis_B < dis_threshold &&
          dis_C < dis_threshold) {
        vote++;
      }
    }
    mylock.lock();
    vote_list[i] = vote;
    mylock.unlock();
  }
  int max_vote_index = 0;
  int max_vote = 0;
  for (size_t i = 0; i < vote_list.size(); i++) {
    if (max_vote < vote_list[i]) {
      max_vote_index = i;
      max_vote = vote_list[i];
    }
  }
  if (max_vote >= 4) {
    auto best_pair = candidate_matcher.match_list_[max_vote_index * skip_len];
    int vote = 0;
    Eigen::Matrix3d best_rot;
    Eigen::Vector3d best_t;
    triangle_solver(best_pair, best_t, best_rot);
    relative_pose.first = best_t;
    relative_pose.second = best_rot;
    for (size_t j = 0; j < candidate_matcher.match_list_.size(); j++) {
      auto verify_pair = candidate_matcher.match_list_[j];
      Eigen::Vector3d A = verify_pair.first.vertex_A_;
      Eigen::Vector3d A_transform = best_rot * A + best_t;
      Eigen::Vector3d B = verify_pair.first.vertex_B_;
      Eigen::Vector3d B_transform = best_rot * B + best_t;
      Eigen::Vector3d C = verify_pair.first.vertex_C_;
      Eigen::Vector3d C_transform = best_rot * C + best_t;
      double dis_A = (A_transform - verify_pair.second.vertex_A_).norm();
      double dis_B = (B_transform - verify_pair.second.vertex_B_).norm();
      double dis_C = (C_transform - verify_pair.second.vertex_C_).norm();
      if (dis_A < dis_threshold && dis_B < dis_threshold &&
          dis_C < dis_threshold) {
        sucess_match_vec.push_back(verify_pair);
      }
    }
    verify_score = plane_geometric_verify(
        plane_cloud_vec_.back(),
        plane_cloud_vec_[candidate_matcher.match_id_.second], relative_pose);
  } else {
    verify_score = -1;
  }
}

void STDescManager::triangle_solver(std::pair<STDesc, STDesc> &std_pair,
                                    Eigen::Vector3d &t, Eigen::Matrix3d &rot) {
  Eigen::Matrix3d src = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d ref = Eigen::Matrix3d::Zero();
  src.col(0) = std_pair.first.vertex_A_ - std_pair.first.center_;
  src.col(1) = std_pair.first.vertex_B_ - std_pair.first.center_;
  src.col(2) = std_pair.first.vertex_C_ - std_pair.first.center_;
  ref.col(0) = std_pair.second.vertex_A_ - std_pair.second.center_;
  ref.col(1) = std_pair.second.vertex_B_ - std_pair.second.center_;
  ref.col(2) = std_pair.second.vertex_C_ - std_pair.second.center_;
  Eigen::Matrix3d covariance = src * ref.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance, Eigen::ComputeThinU |
                                                        Eigen::ComputeThinV);
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d U = svd.matrixU();
  rot = V * U.transpose();
  if (rot.determinant() < 0) {
    Eigen::Matrix3d K;
    K << 1, 0, 0, 0, 1, 0, 0, 0, -1;
    rot = V * K * U.transpose();
  }
  t = -rot * std_pair.first.center_ + std_pair.second.center_;
}

double STDescManager::plane_geometric_verify(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform) {
  Eigen::Vector3d t = transform.first;
  Eigen::Matrix3d rot = transform.second;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(
      new pcl::KdTreeFLANN<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < target_cloud->size(); i++) {
    pcl::PointXYZ pi;
    pi.x = target_cloud->points[i].x;
    pi.y = target_cloud->points[i].y;
    pi.z = target_cloud->points[i].z;
    input_cloud->push_back(pi);
  }
  kd_tree->setInputCloud(input_cloud);
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  double useful_match = 0;
  double normal_threshold = config_setting_.normal_threshold_;
  double dis_threshold = config_setting_.dis_threshold_;
  for (size_t i = 0; i < source_cloud->size(); i++) {
    pcl::PointXYZINormal searchPoint = source_cloud->points[i];
    pcl::PointXYZ use_search_point;
    use_search_point.x = searchPoint.x;
    use_search_point.y = searchPoint.y;
    use_search_point.z = searchPoint.z;
    Eigen::Vector3d pi(searchPoint.x, searchPoint.y, searchPoint.z);
    pi = rot * pi + t;
    use_search_point.x = pi[0];
    use_search_point.y = pi[1];
    use_search_point.z = pi[2];
    Eigen::Vector3d ni(searchPoint.normal_x, searchPoint.normal_y,
                       searchPoint.normal_z);
    ni = rot * ni;
    int K = 3;
    if (kd_tree->nearestKSearch(use_search_point, K, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0) {
      for (size_t j = 0; j < K; j++) {
        pcl::PointXYZINormal nearstPoint =
            target_cloud->points[pointIdxNKNSearch[j]];
        Eigen::Vector3d tpi(nearstPoint.x, nearstPoint.y, nearstPoint.z);
        Eigen::Vector3d tni(nearstPoint.normal_x, nearstPoint.normal_y,
                            nearstPoint.normal_z);
        Eigen::Vector3d normal_inc = ni - tni;
        Eigen::Vector3d normal_add = ni + tni;
        double point_to_plane = fabs(tni.transpose() * (pi - tpi));
        if ((normal_inc.norm() < normal_threshold ||
             normal_add.norm() < normal_threshold) &&
            point_to_plane < dis_threshold) {
          useful_match++;
          break;
        }
      }
    }
  }
  return useful_match / source_cloud->size();
}

void STDescManager::PlaneGeomrtricIcp(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform) {
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(
      new pcl::KdTreeFLANN<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < target_cloud->size(); i++) {
    pcl::PointXYZ pi;
    pi.x = target_cloud->points[i].x;
    pi.y = target_cloud->points[i].y;
    pi.z = target_cloud->points[i].z;
    input_cloud->push_back(pi);
  }
  kd_tree->setInputCloud(input_cloud);
  ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold;
  ceres::Problem problem;
  ceres::LossFunction *loss_function = nullptr;
  Eigen::Matrix3d rot = transform.second;
  Eigen::Quaterniond q(rot);
  Eigen::Vector3d t = transform.first;
  double para_q[4] = {q.x(), q.y(), q.z(), q.w()};
  double para_t[3] = {t(0), t(1), t(2)};
  problem.AddParameterBlock(para_q, 4, quaternion_manifold);
  problem.AddParameterBlock(para_t, 3);
  Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
  Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);
  std::cout << "#######################################"<< std::endl;
  std::cout << "before optimization t[0]:" << para_t[0]
            << ",t[1]" << para_t[1] << ",t[2]" << para_t[2]  << std::endl;
  std::cout << "before optimization qx:" << para_q[0]
            << ",qy" << para_q[1] << ",qz" << para_q[2]  << std::endl;
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  int useful_match = 0;
  for (size_t i = 0; i < source_cloud->size(); i++) {
    pcl::PointXYZINormal searchPoint = source_cloud->points[i];
    Eigen::Vector3d pi(searchPoint.x, searchPoint.y, searchPoint.z);
    pi = rot * pi + t;
    pcl::PointXYZ use_search_point;
    use_search_point.x = pi[0];
    use_search_point.y = pi[1];
    use_search_point.z = pi[2];
    Eigen::Vector3d ni(searchPoint.normal_x, searchPoint.normal_y,
                       searchPoint.normal_z);
    ni = rot * ni;
    if (kd_tree->nearestKSearch(use_search_point, 1, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0) {
      pcl::PointXYZINormal nearstPoint =
          target_cloud->points[pointIdxNKNSearch[0]];
      Eigen::Vector3d tpi(nearstPoint.x, nearstPoint.y, nearstPoint.z);
      Eigen::Vector3d tni(nearstPoint.normal_x, nearstPoint.normal_y,
                          nearstPoint.normal_z);
      Eigen::Vector3d normal_inc = ni - tni;
      Eigen::Vector3d normal_add = ni + tni;
      double point_to_point_dis = (pi - tpi).norm();
      double point_to_plane = fabs(tni.transpose() * (pi - tpi));
      if ((normal_inc.norm() < config_setting_.normal_threshold_ ||
           normal_add.norm() < config_setting_.normal_threshold_) &&
          point_to_plane < config_setting_.dis_threshold_ &&
          point_to_point_dis < 3) {
        useful_match++;
        ceres::CostFunction *cost_function;
        Eigen::Vector3d curr_point(source_cloud->points[i].x,
                                   source_cloud->points[i].y,
                                   source_cloud->points[i].z);
        Eigen::Vector3d curr_normal(source_cloud->points[i].normal_x,
                                    source_cloud->points[i].normal_y,
                                    source_cloud->points[i].normal_z);

        cost_function = PlaneSolver::Create(curr_point, curr_normal, tpi, tni);
        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
      }
    }
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  Eigen::Quaterniond q_opt(para_q[3], para_q[0], para_q[1], para_q[2]);
  rot = q_opt.toRotationMatrix();
  t << t_last_curr(0), t_last_curr(1), t_last_curr(2);
  std::cout << "///////////////////////////////////////////"<< std::endl;
  std::cout << "after optimization t[0]:" << t[0]
            << ",t[1]" << t[1] << ",t[2]" << t[2]  << std::endl;
  std::cout << "after optimization qx:" << para_q[0]
            << ",qy" << para_q[1] << ",qz" << para_q[2]  << std::endl;
  transform.first = t;
  transform.second = rot;
  // std::cout << "useful match for icp:" << useful_match << std::endl;
}

void OctoTree::init_plane() {
  plane_ptr_->covariance_ = Eigen::Matrix3d::Zero();
  plane_ptr_->center_ = Eigen::Vector3d::Zero();
  plane_ptr_->normal_ = Eigen::Vector3d::Zero();
  plane_ptr_->points_size_ = voxel_points_.size();
  plane_ptr_->radius_ = 0;
  for (auto pi : voxel_points_) {
    plane_ptr_->covariance_ += pi * pi.transpose();
    plane_ptr_->center_ += pi;
  }
  plane_ptr_->center_ = plane_ptr_->center_ / plane_ptr_->points_size_;
  plane_ptr_->covariance_ =
      plane_ptr_->covariance_ / plane_ptr_->points_size_ -
      plane_ptr_->center_ * plane_ptr_->center_.transpose();
  Eigen::EigenSolver<Eigen::Matrix3d> es(plane_ptr_->covariance_);
  Eigen::Matrix3cd evecs = es.eigenvectors();
  Eigen::Vector3cd evals = es.eigenvalues();
  Eigen::Vector3d evalsReal;
  evalsReal = evals.real();
  Eigen::Matrix3d::Index evalsMin, evalsMax;
  evalsReal.rowwise().sum().minCoeff(&evalsMin);
  evalsReal.rowwise().sum().maxCoeff(&evalsMax);
  int evalsMid = 3 - evalsMin - evalsMax;
  if (evalsReal(evalsMin) < config_setting_.plane_detection_thre_) {
    plane_ptr_->normal_ << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
        evecs.real()(2, evalsMin);
    plane_ptr_->min_eigen_value_ = evalsReal(evalsMin);
    plane_ptr_->radius_ = sqrt(evalsReal(evalsMax));
    plane_ptr_->is_plane_ = true;

    plane_ptr_->intercept_ = -(plane_ptr_->normal_(0) * plane_ptr_->center_(0) +
                               plane_ptr_->normal_(1) * plane_ptr_->center_(1) +
                               plane_ptr_->normal_(2) * plane_ptr_->center_(2));
    plane_ptr_->p_center_.x = plane_ptr_->center_(0);
    plane_ptr_->p_center_.y = plane_ptr_->center_(1);
    plane_ptr_->p_center_.z = plane_ptr_->center_(2);
    plane_ptr_->p_center_.normal_x = plane_ptr_->normal_(0);
    plane_ptr_->p_center_.normal_y = plane_ptr_->normal_(1);
    plane_ptr_->p_center_.normal_z = plane_ptr_->normal_(2);
  } else {
    plane_ptr_->is_plane_ = false;
  }
}

void OctoTree::init_octo_tree() {
  if (voxel_points_.size() > config_setting_.voxel_init_num_) {
    init_plane();
  }
}

