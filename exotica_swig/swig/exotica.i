%module exotica

%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::Vector2d)
%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::VectorXi)

%{
#include "exotica/Property.h"
using namespace exotica;
%}

%include boost_shared_ptr.i

%include "exotica/Property.h"
%include "exotica/SceneInitializer.h"
%template(InstantiableSceneInitializer) exotica::Instantiable<exotica::SceneInitializer>;

%ignore Property;
%ignore Uncopyable;
%ignore exotica::Instantiable<exotica::SceneInitializer>;



%{
#include "exotica/Exotica.h"
%}

%shared_ptr(exotica::CollisionScene);

%include "exotica/Object.h"
%include "exotica/Tools/Uncopyable.h"
%include "exotica/Version.h"
%include "exotica/Scene.h"



