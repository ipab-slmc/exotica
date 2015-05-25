/***********************************************************************\
|    This component provides a set of common functionalities for        |
 |   EXOTica:                                                            |
 |     1) Debugging macros                                               |
 |     2) Enum for error-propagation/reporting                           |
 |     3) XML-Parsing functionality                                      |
 |                                                                       |
 |           Developer: Michael Camilleri (mcamnadur@gmail.com)          |
 |                    Last Edited: 14 - March - 2014                     |
 \***********************************************************************/

#ifndef EXOTICA_TOOLS_H
#define EXOTICA_TOOLS_H

#include "tinyxml2/tinyxml2.h"
#include <Eigen/Dense>
#include <kdl/tree.hpp>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "rapidjson/document.h"
#include <exotica/MeshVertex.h>
#include <exotica/StringList.h>
#include <exotica/BoolList.h>
#include <exotica/Vector.h>
#include <exotica/Matrix.h>

/**
 * \brief A set of debugging tools: basically these provide easy ways of checking code execution through std::cout prints
 */
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif

#ifdef EXOTICA_DEBUG_MODE
#define CHECK_EXECUTION         std::cout << "\033[1;32m[EXOTica]:\033[0m Ok in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n"; //!< With endline
#define INDICATE_FAILURE        std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\033[0m\n";//!< With endline
#define WARNING(x)				std::clog << "\033[1;32m[EXOTica]:\033[0m \033[33mWarning in " << __PRETTY_FUNCTION__ << ": " << x << "\033[0m\n";//!< With endline
#define ERROR(x)				std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n" << x << "\033[0m\n";//!< With endline
#define INFO(x)					std::clog << "\033[1;32m[EXOTica]:\033[0m Info in " << __PRETTY_FUNCTION__ << ": " << x << "\n";//!< With endline
#else
#define CHECK_EXECUTION         // No operation
#define INDICATE_FAILURE        std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\033[0m\n";//!< With endline
#define WARNING(x)
#define ERROR(x)                std::cerr << "\033[1;32m[EXOTica]:\033[0m \033[1;31mFailed in " << __FILE__ << " at line " << __LINE__ << " within function " << __PRETTY_FUNCTION__ << ".\n" << x << "\033[0m\n";//!< With endline
#define INFO(x)
#endif
#define HIGHLIGHT(x)			std::cout << "\033[1;32m[EXOTica]:\033[0m \033[36m" << x << "\033[0m\n";
#define HIGHLIGHT_NAMED(name, x)std::cout << "\033[1;32m[EXOTica]:\033[0m \033[35m[" << name <<"]\033[0m \033[36m" << x << "\033[0m\n";
#define WARNING_NAMED(name, x)	std::cout << "\033[1;32m[EXOTica]:\033[0m \033[35m[" << name <<"]\033[0m \033[33m" << x << "\033[0m\n";
#define INFO_NAMED(name, x)		std::cout << "\033[1;32m[EXOTica]:\033[0m \033[35m[" << name <<"]\033[0m " << x << "\n";
namespace Eigen
{

	/// \brief Convenience wrapper for storing references to sub-matrices/vectors
	template<typename Derived>
	class Ref_ptr: public boost::shared_ptr<Ref<Derived> >
	{
		public:
			inline Ref_ptr() :
					boost::shared_ptr<Ref<Derived> >()
			{

			}

			inline Ref_ptr(const Eigen::Block<Derived>& other)
			{
				this->reset(new Ref<Derived>(other));
			}

			inline Ref_ptr(Eigen::Block<Derived>& other)
			{
				this->reset(new Ref<Derived>(other));
			}

			inline Ref_ptr(const Eigen::VectorBlock<Derived>& other)
			{
				this->reset(new Ref<Derived>(other));
			}

			inline Ref_ptr(Eigen::VectorBlock<Derived>& other)
			{
				this->reset(new Ref<Derived>(other));
			}

			inline Ref_ptr(Derived& other)
			{
				this->reset(new Ref<Derived>(other));
			}

			inline Ref_ptr(const Derived& other)
			{
				this->reset(new Ref<Derived>(other));
			}
	};

	/// \brief Reference to sub-vector.
	typedef Ref_ptr<VectorXd> VectorXdRef_ptr;
	/// \brief Reference to sub-Matrix.
	typedef Ref_ptr<MatrixXd> MatrixXdRef_ptr;

	typedef Ref<VectorXd> VectorXdRef;
	typedef const Ref<const VectorXd>& VectorXdRefConst;
	typedef Ref<MatrixXd> MatrixXdRef;
	typedef const Ref<const MatrixXd>& MatrixXdRefConst;
}

/**
 * \brief A convenience macro for the boost scoped lock
 */
#define LOCK(x) boost::mutex::scoped_lock(x)

/**
 * \brief A double-wrapper MACRO functionality for generating unique object names: The actual functionality is provided by EX_UNIQ (for 'exotica unique')
 */
#define EX_CONC(x, y) x ## y
#define EX_UNIQ(x, y) EX_CONC(x, y)

namespace exotica
{
	/**
	 * \brief Enum for error reporting throughout the library
	 */
	enum EReturn
	{
		SUCCESS = 0,  //!< Indicates successful execution of function
		PAR_INV = 1,  //!< Invalid Parameter TYPE! (when using dynamic polymorphism)
		PAR_ERR = 2, //!< Uninitialised or incorrect parameter value (could be sizes of vectors, nan etc...)
		MMB_NIN = 3,  //!< A member required by this function is Not INititialised correctly
		MEM_ERR = 4,  //!< A memory error (for example when creating a new class)
		WARNING = 50, //!< A generic warning:
		FAILURE = 100, //!< Indicates a generic failure
		CANCELLED = 200 //!< The process has been successful but the results should be ignored
	};

	/**
	 * \brief	Enum for termination criterion
	 */
	enum ETerminate
	{
		TERMINATE = 0, CONTINUE = 1
	};

	bool ok(const EReturn & value);

	/**
	 * \brief Parses an XML element into an Eigen Matrix. The handle must point directly to the element with the matrix as its text child and must have no comments!
	 * @param xml_matrix    The element for the XML matrix
	 * @param eigen_matrix  Placeholder for storing the parsed matrix
	 * @return              Indication of success: TODO
	 */
	EReturn getMatrix(const tinyxml2::XMLElement & xml_matrix, Eigen::MatrixXd & eigen_matrix);

	/**
	 * \brief Parses an XML element into an Eigen Vector. The handle must point directly to the element with the vector as its text child and must have no comments!
	 * @param xml_matrix    The element for the XML Vector
	 * @param eigen_matrix  Placeholder for storing the parsed vector
	 * @return              Indication of success: TODO
	 */
	EReturn getVector(const tinyxml2::XMLElement & xml_vector, Eigen::VectorXd & eigen_vector);
	EReturn getStdVector(const tinyxml2::XMLElement & xml_vector, std::vector<double> & std_vector);
	EReturn getBoolVector(const tinyxml2::XMLElement & xml_vector, std::vector<bool> & bool_vector);

	/**
	 * \brief Get boolean
	 */
	EReturn getBool(const tinyxml2::XMLElement & xml_vector, bool & val);

	/**
	 * \brief Parses an XML element into an float (for convenience)
	 * @param xml_value    The element containing the numerical value as text child
	 * @param value  Placeholder for storing the parsed double
	 * @return              Indication of success: TODO
	 */
	EReturn getDouble(const tinyxml2::XMLElement & xml_value, double & value);

	/**
	 * \brief Parses an XML element into an float (for convenience)
	 * @param xml_value    The element containing the numerical value as text child
	 * @param value  Placeholder for storing the parsed integer
	 * @return              Indication of success: TODO
	 */
	EReturn getInt(const tinyxml2::XMLElement & xml_value, int & value);

	/**
	 * \brief Parses an XML element into a vector of string
	 * @param xml_value    The element containing the numerical value as text child
	 * @param value  Placeholder for storing the parsed list
	 * @return              Indication of success: TODO
	 */
	EReturn getList(const tinyxml2::XMLElement & xml_value, std::vector<std::string> & value);

	/**
	 * \brief Removes all characters after the last forward slash (/): intended to be used to get a parent directory from a file-path
	 * @param file_path[inout]  The complete file-path to be processed: the returned string contains no trailing forward slashes...
	 * @return                  SUCCESS if everything ok
	 *                          @n WARNING if already in root directory
	 *                          @n PAR_ERR if string is incorrectly formed.
	 */
	EReturn resolveParent(std::string & file_path);

	/**
	 * \brief Utility function for including files:
	 *        Iterates through all children looking for include tags, recursively calling itself until leaves are reached
	 * @param handle    Handle to the current element: will modify the document it is associated with...
	 * @param directory The current working directory
	 * @return          Indication of success TODO
	 */
	EReturn parseIncludes(tinyxml2::XMLHandle & handle, std::string directory);

	/**
	 * \brief Utility function for copying a complete sub-tree from one document to another
	 * @param parent[inout] The parent-to-be, where the sub-tree will be copied
	 * @param child[in]     The next child to add to this parent...
	 * @return              Indication of SUCCESS TODO
	 */
	EReturn deepCopy(tinyxml2::XMLHandle & parent, tinyxml2::XMLHandle & child);

	/**
	 * @brief loadOBJ Loads mesh data from an OBJ file
	 * @param file_name File name
	 * @param tri Returned vertex indices of triangles
	 * @param vert Vertex positions
	 * @return Indication of SUCCESS
	 */
	EReturn loadOBJ(std::string & data, Eigen::VectorXi& tri, Eigen::VectorXd& vert);

	EReturn saveMatrix(std::string file_name, const Eigen::Ref<const Eigen::MatrixXd> mat);

    EReturn getJSON(const rapidjson::Value& a, Eigen::VectorXd& ret);
    EReturn getJSON(const rapidjson::Value& a, double& ret);
    EReturn getJSON(const rapidjson::Value& a, int& ret);
    EReturn getJSON(const rapidjson::Value& a, std::string& ret);
    EReturn getJSON(const rapidjson::Value& a, KDL::Frame& ret);
    EReturn getJSON(const rapidjson::Value& a, std::vector<std::string>& ret);

	EReturn vectorExoticaToEigen(const exotica::Vector & exotica, Eigen::VectorXd & eigen);
	EReturn vectorEigenToExotica(Eigen::VectorXd eigen, exotica::Vector & exotica);
	EReturn matrixExoticaToEigen(const exotica::Matrix & exotica, Eigen::MatrixXd & eigen);
	EReturn matrixEigenToExotica(const Eigen::MatrixXd & eigen, exotica::Matrix & exotica);
}
#endif
