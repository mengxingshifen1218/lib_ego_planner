#ifndef _HEADER_H_
#define _HEADER_H_

#include <vector>
#include <mutex>
#include <fstream>
#include <memory>

#define OPENCV

#ifdef OPENCV
#include "opencv2/opencv.hpp"
#endif


using namespace std;



#define MIN_MAP_WIDTH  90
#define MIN_MAP_HEIGHT 90
#define MIN_MAP_SIZE (MIN_MAP_WIDTH * MIN_MAP_HEIGHT)

#define EXTEND_MORE_DIS  2.0f

#define DEFAULT_RESOLUTION 0.05f

#define MAX_TIME 120

#pragma pack(push, 1)

typedef struct _TMapParam {
public:
    _TMapParam()
    {
        width = MIN_MAP_WIDTH;
        height = MIN_MAP_HEIGHT;
        resolution = DEFAULT_RESOLUTION;
        xMin = -MIN_MAP_WIDTH * resolution*0.5;
        yMin = -MIN_MAP_HEIGHT * resolution*0.5;
        //min align pixel
        xMin = ((int)(xMin / resolution + 0.5f))*resolution;
        yMin = ((int)(yMin / resolution + 0.5f))*resolution;
    }
    _TMapParam(float minX, float minY, int w, int h)
    {
        width = w;
        height = h;
        resolution = DEFAULT_RESOLUTION;
        xMin = minX;
        yMin = minY;
        //min align pixel
        xMin = ((int)(xMin / resolution + 0.5f))*resolution;
        yMin = ((int)(yMin / resolution + 0.5f))*resolution;
    }
public:
    int   width;
    int   height;
    float resolution;
    float xMin;
    float yMin;
    int x2idx(float x) const
    {
        int tempX = static_cast<int>((x - xMin) / resolution/* + 0.5f*/);  //float 转 int 去尾
        tempX = tempX < width - 1 ? tempX : width - 1;//防止溢出
        tempX = tempX > 0 ? tempX : 0;
        return tempX;
    }
    int y2idx(float y) const
    {
        int tempY = static_cast<int>((y - yMin) / resolution/* + 0.5f*/);  //float 转 int 去尾
        tempY = tempY < height - 1 ? tempY : height - 1;//防止溢出
        tempY = tempY > 0 ? tempY : 0;
        return tempY;
    }
    float idx2x(const unsigned int cx) const { return xMin + (cx + 0.5f)*resolution; }//加上半个像素
    float idx2y(const unsigned int cy) const { return yMin + (cy + 0.5f)*resolution; }//加上半个像素
}TMapParam;

#pragma pack(pop)

typedef struct _TMapData {
    TMapParam mapParam;
    std::vector<unsigned char> map;	
    //拷贝构造函数
    _TMapData(const _TMapData& copy)
    {
        mapParam = copy.mapParam;
        map = copy.map;
    }

    _TMapData()
    {
        mapParam = TMapParam();
        map.resize(mapParam.width*mapParam.height);
        memset(&map[0], 0, map.size());
    }

    ~_TMapData()
    {
        std::vector<unsigned char>().swap(map);
    }

    _TMapData(TMapParam param, uint8_t val)
    {
        mapParam = param;
        map.resize(mapParam.width*mapParam.height, val);
    }

    inline int   x2idx(float x) const
    {
        int tempX = static_cast<int>((x - mapParam.xMin) / mapParam.resolution/* + 0.5f*/);
        tempX = tempX < mapParam.width - 1 ? tempX : mapParam.width - 1;//防止溢出
        tempX = tempX > 0 ? tempX : 0;
        return tempX;
    }
    inline int   y2idx(float y) const
    {
        int tempY = static_cast<int>((y - mapParam.yMin) / mapParam.resolution/* + 0.5f*/);
        tempY = tempY < mapParam.height - 1 ? tempY : mapParam.height - 1;//防止溢出
        tempY = tempY > 0 ? tempY : 0;
        return tempY;
    }

    inline float   idx2x(const unsigned int cx) const { return mapParam.xMin + (cx + 0.5f)*mapParam.resolution; }
    inline float   idx2y(const unsigned int cy) const { return mapParam.yMin + (cy + 0.5f)*mapParam.resolution; }

    //get TMapData size
    int GetDataSize()
    {
        int mapDataSize = sizeof(TMapParam) + mapParam.width * mapParam.height * sizeof(unsigned char);
        return mapDataSize;
    }

    bool ReadTMapDataAndMapIdFromRawDataFile(const char *path, uint64_t *mapId) {
        ifstream fin;
        fin.open(path, ios_base::in | ios_base::binary);

        if (fin.is_open()) {
            fin.read((char *)mapId, sizeof(uint64_t));
            fin.read((char *)&mapParam, sizeof(TMapParam));
            map.resize(mapParam.width*mapParam.height);
            fin.read((char *)&map[0], mapParam.width*mapParam.height);
            fin.close();
            return true;
        }
        return false;
    }

    bool WriteTMapDataAndMapIdToRawDataFile(const char *path, uint64_t *mapId) {
        ofstream ofs;
        ofs.open(path, ios_base::out | ios_base::binary);
        if (ofs.is_open()) {
            ofs.write((char *)mapId, sizeof(uint64_t));
            ofs.write((char *)&mapParam, sizeof(TMapParam));
            ofs.write((char *)&map[0], mapParam.width*mapParam.height);
            ofs.flush();
            ofs.close();
            return true;
        }
        return false;
    }

	bool ReadTMapDataFromRawDataFile(const char *path) {
		//LOGD("ReadTMapDataFromRawDataFile...");
		ifstream fin;
		fin.open(path, ios_base::in | ios_base::binary);

		if (fin.is_open()) {
			fin.read((char *)&mapParam, sizeof(TMapParam));
			map.resize(mapParam.width*mapParam.height);
			fin.read((char *)&map[0], mapParam.width*mapParam.height);
			fin.close();
			//LOGD("ReadTMapDataFromRawDataFile Done.");
			return true;
		}
		return false;
	}

	bool WriteTMapDataToRawDataFile(const char *path) {
		//Write to file
		//LOGD("LoadTMapDataFromeRawDataFile...");
		ofstream ofs;
		ofs.open(path, ios_base::out | ios_base::binary);
		if (ofs.is_open()) {
			ofs.write((char *)&mapParam, sizeof(TMapParam));
			ofs.write((char *)&map[0], mapParam.width*mapParam.height);
			ofs.flush();
			ofs.close();
			//LOGD("SaveTMapDataToRawDataFile Done.");
			return true;
		}
		//SaveTMapDataToFile(&mTmapShow, path);
		return false;
	}

    bool WritePgm(const char* fname, bool is_plainPGM)
    {
        int width = mapParam.width, height = mapParam.height;
        unsigned char* data = new unsigned char[width*height];
    	std::memcpy(data, &map[0], mapParam.height* mapParam.width);

        FILE* fid = nullptr;
        if (is_plainPGM) {
            fid = fopen(fname, "w");
            if (!fid) return false;
            fprintf(fid, "P2\n%d %d\n255\n", width, height);
            for (int i = height - 1; i >= 0 ; i--)
                for (int j = 0; j < width; j++)
                    fprintf(fid, "%d\n", data[i*width + j]);
        } else {
            fid = fopen(fname, "wb");
            if (!fid) return false;
            fprintf(fid, "P5\n%d %d\n255\n", width, height);
            for (int i = 0; i < height; i++)
                fwrite(data + i*width, 1, width, fid);
        }
        delete[] data;
        fclose(fid);
        return true;
    }
 

    void GetPixelCount(size_t* whitePixelCnt, size_t* grayPixelCnt, size_t* blackPixelCnt)
    {
        size_t bCnt =0, wCnt = 0, gCnt = 0;
        for(size_t i=0;i<map.size(); i++){
            if(map[i] < 50)     bCnt++;
            if(map[i] > 100 && map[i] < 150)    gCnt++;
            if(map[i] > 200)    wCnt ++;
        }
        if(whitePixelCnt != NULL)   *whitePixelCnt = wCnt;
        if(blackPixelCnt != NULL)   *blackPixelCnt = bCnt;
        if(grayPixelCnt != NULL)    *grayPixelCnt = gCnt;
    }

#ifdef OPENCV
    _TMapData(cv::Mat grayPic)
    {
        mapParam.width = grayPic.cols;
        mapParam.height = grayPic.rows;
        mapParam.resolution = DEFAULT_RESOLUTION;
        mapParam.xMin = 0;
        mapParam.yMin = 0;
        map.resize(mapParam.height* mapParam.width);
        std::memcpy( &map[0], grayPic.data, mapParam.height* mapParam.width);
    }

	cv::Mat GetOpencvMat() {
		cv::Mat re(mapParam.height, mapParam.width,CV_8U);
		std::memcpy(re.data,&map[0], mapParam.height* mapParam.width);
		return re;
	}
#endif

}TMapData;


#endif
