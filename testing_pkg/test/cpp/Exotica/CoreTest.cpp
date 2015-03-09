#include "testing_pkg/Exotica/CoreTest.h"

using namespace testing;

TEST(ExoticaCoreTest, ECT)  //!< Exotica Core Test
{
  //!< Create types
  exotica::Object ObjectTest;
  DClass dClassTest;
  
  //!< Check for polymorphic behaviour of the Base Object object
  EXPECT_EQ(0, ObjectTest.type().compare("exotica::Object")) << ObjectTest.type();
  EXPECT_EQ(0, dClassTest.type().compare("testing::DClass")) << dClassTest.type();
  EXPECT_EQ(0, dClassTest.name.compare("TestingDerived")) << dClassTest.name;
  //!< Now repeat with pointers to the Base and Derived classes
  exotica::Object * base_ptr = &ObjectTest;
  EXPECT_EQ(0, base_ptr->type().compare("exotica::Object")) << base_ptr->type();
  base_ptr = &dClassTest;
  EXPECT_EQ(0, base_ptr->type().compare("testing::DClass")) << base_ptr->type();
  
  //!< Check for the Object Factory functions
  std::vector<std::string> class_registrar;
  exotica::EReturn returntype = exotica::Factory< std::string, exotica::Object >::Instance().listImplementations(class_registrar);
  ASSERT_EQ(exotica::SUCCESS, returntype);
  EXPECT_GE(2, class_registrar.size()); //!< Should be at least two classes registered
  bool e_ok = false;
  bool d_ok = false;
  for (int i=0; i<class_registrar.size(); i++)
  {
    if (class_registrar[i].compare("ObjectTest") == 0) { e_ok = true; }
    if (class_registrar[i].compare("DClassTest") == 0) { d_ok = true; }
  }
  ASSERT_TRUE(e_ok);
  ASSERT_TRUE(d_ok);
  EXPECT_EQ(exotica::PAR_ERR, (exotica::Factory< std::string, exotica::Object >::Instance().registerType("DClassTest", [] () -> exotica::Object * { return new DClass; })));
  EXPECT_EQ(exotica::SUCCESS, (exotica::Factory< std::string, exotica::Object >::Instance().registerType("DClassTest2", [] () -> exotica::Object * { return new DClass; })));

  //!< Check for the Test-Factory functions
  std::string temp_string;
  ASSERT_EQ(exotica::PAR_ERR, (exotica::XMLTester::Instance().getTest("ObjectTest", temp_string))); //!< There should be no instance of this recorded
  ASSERT_EQ(exotica::SUCCESS, (exotica::XMLTester::Instance().getTest("DClassTest", temp_string)));
  EXPECT_EQ(0, temp_string.compare("DerivedXML"));
  
  //!< Check for Object Creation
  boost::shared_ptr<exotica::Object> temp_ptr;
  ASSERT_EQ(exotica::PAR_ERR, (exotica::Factory< std::string, exotica::Object >::Instance().createObject("WrongName", temp_ptr)));
  ASSERT_EQ(exotica::SUCCESS, (exotica::Factory< std::string, exotica::Object >::Instance().createObject("DClassTest", temp_ptr)));
  ASSERT_NE(nullptr, temp_ptr);
  EXPECT_EQ(0, boost::dynamic_pointer_cast<DClass>(temp_ptr)->name.compare("TestingDerived"));
}

TEST(ExoticaCoreTest, EPT)  //!< Exotica Parser Test
{
  //!< Initialise
  std::string resource_path;
  ASSERT_TRUE(findPackagePath("testing_pkg", resource_path));  //!< Removes dependency on ros (in the future)
  resource_path.append("/resource/Exotica/");
   
      
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, doc.LoadFile((resource_path + std::string("Parsing.xml")).c_str()));
  tinyxml2::XMLHandle handle(doc.RootElement());  //!< Get a handle to the root element
   
  handle = handle.FirstChildElement("Matrix");
  Eigen::MatrixXd temp_matrix;
   
  for (int i=0; i<7; i++) //!< Handle the wrong ones
  {
     
    if (!handle.ToElement()) { ADD_FAILURE() << "Could Not find element"; break; }
     
    EXPECT_EQ(exotica::PAR_ERR, exotica::getMatrix(*(handle.ToElement()), temp_matrix));
     
    handle = handle.NextSiblingElement("Matrix");
     
  }
  //!< Now we should be pointing to the last one, which is correct
  if (!handle.ToElement())
  {
    ADD_FAILURE() << "Could Not find element";
  }
  else
  {
    EXPECT_EQ(exotica::SUCCESS, exotica::getMatrix(*(handle.ToElement()), temp_matrix));
     
    Eigen::MatrixXd default_matrix(3,3);
     
    default_matrix << 0.5, 0.8, 0.1, 0.0, 0.0, 1.2, 0.6, 0.7, 0.01;
     
    EXPECT_TRUE(compareMatrices(default_matrix, temp_matrix));
     
  }
   
  //!< Now the vector
  handle = tinyxml2::XMLHandle(doc.RootElement());  //!< Go back to the root
  handle = handle.FirstChildElement("Vector");
  Eigen::VectorXd temp_vector;
   
  for (int i=0; i<2; i++)
  {
    if (!handle.ToElement()) { ADD_FAILURE() << "Could Not find element"; break; }
    EXPECT_EQ(exotica::PAR_ERR, exotica::getVector(*(handle.ToElement()), temp_vector));
    handle = handle.NextSiblingElement("Vector");
  }
   
  //!< Now we should be pointing to the last one, which is correct
  if (!handle.ToElement())
  {
    ADD_FAILURE() << "Could Not find element";
  }
  else
  {
    EXPECT_EQ(exotica::SUCCESS, exotica::getVector(*(handle.ToElement()), temp_vector)) << temp_vector;
    Eigen::VectorXd default_vector(3);
    default_vector << 0.1, 0.2, 0.3;
    EXPECT_TRUE(compareVectors(default_vector, temp_vector));
  }
  
  //!< And finally the scalar value
  handle = tinyxml2::XMLHandle(doc.RootElement());  //!< Go back to the root
  handle = handle.FirstChildElement("Scalar");
  double temp_scalar;
  for (int i=0; i<2; i++)
  {
    if (!handle.ToElement()) { ADD_FAILURE() << "Could Not find element"; break; }
    EXPECT_EQ(exotica::PAR_ERR, exotica::getDouble(*(handle.ToElement()), temp_scalar)) << " @ " << i;
    handle = handle.NextSiblingElement("Scalar");
  }
  
  if (!handle.ToElement())
  {
    ADD_FAILURE() << "Could Not find element";
  }
  else
  {
    EXPECT_EQ(exotica::SUCCESS, exotica::getDouble(*(handle.ToElement()), temp_scalar)) << temp_scalar;
    EXPECT_FLOAT_EQ(0.5123, temp_scalar);
  }
}

TEST(ExoticaCoreTest, SPT)  //!< String Path parsing test
{
  std::string file_path = "/hello/this/is/me/testing.yay";
  
  //!< Correct string
  EXPECT_EQ(exotica::SUCCESS, exotica::resolveParent(file_path));
  EXPECT_EQ(0, file_path.compare("/hello/this/is/me")) << file_path;
  
  //!< Already in Root directory
  file_path = "/hello";
  EXPECT_EQ(exotica::WARNING, exotica::resolveParent(file_path));
  EXPECT_EQ(0, file_path.compare("/"));
  
  //!< File path contains trailing "/"
  file_path = "/hello/this/is/me/";
  EXPECT_EQ(exotica::SUCCESS, exotica::resolveParent(file_path));
  EXPECT_EQ(0, file_path.compare("/hello/this/is/me")) << file_path;
  
  //!< Invalid File path type
  file_path = "My name is Michael Camilleri";
  EXPECT_EQ(exotica::PAR_ERR, exotica::resolveParent(file_path));
  EXPECT_EQ(0, file_path.compare("My name is Michael Camilleri"));
}

TEST(ExoticaCoreTest, XCT)  //!< XML Copy tests
{
  std::string resource_path;
  ASSERT_TRUE(findPackagePath("testing_pkg", resource_path));  //!< Removes dependency on ros (in the future)
  resource_path.append("/resource/Exotica/");
  
  //!< Open the documents
  tinyxml2::XMLDocument clone_1, clone_2;
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, clone_1.LoadFile((resource_path + std::string("clone_1.xml")).c_str()));
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, clone_2.LoadFile((resource_path + std::string("clone_2.xml")).c_str()));
  
  //!< Assign handles
  tinyxml2::XMLHandle handle_1(clone_1.RootElement());
  tinyxml2::XMLHandle handle_2(clone_2.RootElement());
  
  //!< Now perform deepcopy
  handle_1 = handle_1.FirstChildElement("include");
  EXPECT_EQ(exotica::SUCCESS, exotica::deepCopy(handle_1, handle_2));
}


TEST(ExoticaCoreTest, XIT)  //!< XML include tests
{
  //!< Temporaries
  std::string resource_path;
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLHandle handle(doc);
  
  //!< Find path
  ASSERT_TRUE(findPackagePath("testing_pkg", resource_path));  //!< Removes dependency on ros (in the future)
  resource_path.append("/resource/Exotica/IncludeTest/");

  //!< Test1: Correct structure
  ASSERT_EQ(tinyxml2::XML_NO_ERROR, doc.LoadFile((resource_path + std::string("top_1.xml")).c_str()));
  handle = tinyxml2::XMLHandle(doc.RootElement());
  ASSERT_EQ(exotica::SUCCESS, exotica::parseIncludes(handle, resource_path));  
}
