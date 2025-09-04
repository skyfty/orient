#ifndef URDF__MODEL_H_
#define URDF__MODEL_H_

#include <string>

#include <urdf_model/model.h>
namespace urdf
{

class Model : public ModelInterface
{
public:
  bool initFile(const std::string & filename);
  bool initString(const std::string & xmlstring);
};

}  // namespace urdf

#endif  // URDF__MODEL_H_
