#ifndef SAVE_FILE_H
#define SAVE_FILE_H

#include <Configuration.h>
#include <Types/cppTypes.h>
#include <stdio.h>
#include <sys/stat.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <list>
#include <string>

template <typename T>
class SaveFileManager
{

public:
  SaveFileManager(const std::string _file_name, const std::string _folder_name);
  ~SaveFileManager(){}

  void cleaning_file();
  void create_folder();  
  
  void saveVector(const DVec<T>& _vec);
  void saveVector(const Vec3<T>& _vec);
  void saveVector(const Vec4<T>& _vec);
  void saveVector(const std::vector<T>& _vec);
  void saveVector(T* _vec, int size);
  void saveMatrix(const DMat<T>& _mat);
  void saveMatrix(const RotMat<T> _mat);
  void saveValue(std::string _value);
  void saveValue(int _value);
  void saveValue(float _value);


private:
  std::string file_name, folder_name, full_ret_file;
  std::ofstream savefile;

  std::list<std::string> gs_fileName_string;  // global & static

};

#endif