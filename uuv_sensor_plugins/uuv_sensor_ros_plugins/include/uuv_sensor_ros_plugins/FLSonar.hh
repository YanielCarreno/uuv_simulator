// Copyright 2018 Brazilian Intitute of Robotics"

/*
This is a slightly modified code of the Forward-Looking Sonar developed by the Brazilian Institue of Robotics here:
https://github.com/Brazilian-Institute-of-Robotics/forward_looking_sonar_gazebo. Credits mainly goes to them.
*/

#ifndef _GAZEBO_RENDERING_SONAR_HH_
#define _GAZEBO_RENDERING_SONAR_HH_

#include <memory>
#include <string>
#include <vector>

#include <sdf/sdf.hh>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"
#include <uuv_sensor_ros_plugins_msgs/SonarStamped.h>

#include <gazebo/physics/physics.hh>

// OpenCV includes
#include <opencv2/opencv.hpp>


namespace Ogre
{
class Material;
class Renderable;
class Pass;
class AutoParamDataSource;
class Matrix4;
class MovableObject;
}

namespace gazebo
{
namespace common
{
class Mesh;
}

namespace rendering
{

/// \addtogroup gazebo_rendering Rendering
/// \{

/// \class Sonar Sonar.hh rendering/rendering.hh
/// \brief GPU based laser distance sensor
class GZ_RENDERING_VISIBLE FLSonar
  : public Camera, public Ogre::RenderObjectListener //makes FLSonar class multi-inherit from Camera and RenderObjectListener classes
{
  /// \brief Constructor
  /// \param[in] _namePrefix Unique prefix name for the camera.
  /// \param[in] _scene Scene that will contain the camera
  /// \param[in] _autoRender Almost everyone should leave this as true.
public:
  FLSonar(const std::string &_namePrefix,
          ScenePtr _scene, const bool _autoRender = true);

  /// \brief Destructor
  virtual ~FLSonar();

  // Documentation inherited
  virtual void Load(sdf::ElementPtr _sdf);

  // Documentation inherited
  virtual void Load();

  // Documentation inherited
  virtual void Init();

  // Documentation inherited
  virtual void Fini();

  /// \brief Create the texture which is used to render laser data.
  /// \param[in] _textureName Name of the new texture.
  void CreateTexture(const std::string &_textureName);

  // Documentation inherited
  virtual void PostRender();


  /// \internal
  /// \brief Implementation of Ogre::RenderObjectListener
  virtual void notifyRenderSingleObject(Ogre::Renderable *_rend,
                                        const Ogre::Pass *_p, const Ogre::AutoParamDataSource *_s,
                                        const Ogre::LightList *_ll, bool _supp);

  /// \brief Get the vertical field-of-view.
  /// \return The vertical field of view of the laser sensor.
  /// \deprecated See VertFOV()
  double GetVertFOV() const;

  /// \brief Get the vertical field-of-view.
  /// \return The vertical field of view of the laser sensor.
  double VertFOV() const;

  /// \brief Get the horizontal field-of-view.
  /// \return The horizontal field of view of the sonar sensor.
  /// \deprecated See HorzFOV()
  double GetHorzFOV() const;

  /// \brief Get the horizontal field-of-view.
  /// \return The horizontal field of view of the sonar sensor.
  double HorzFOV() const;

  /// \brief Set the horizontal fov
  /// \param[in] _hfov horizontal fov
  void SetHorzFOV(const double _hfov);

  /// \brief Get near clip
  /// \return near clip distance
  /// \deprecated See NearClip()
  double GetNearClip() const;

  /// \brief Get near clip
  /// \return near clip distance
  double NearClip() const;

  /// \brief Get far clip
  /// \return far clip distance
  /// \deprecated See FarClip()
  double GetFarClip() const;

  /// \brief Get far clip
  /// \return far clip distance
  double FarClip() const;

  /// \brief Get the shader output
  /// \return shader output
  cv::Mat ShaderImage() const;

  /// \brief Get the sonar image on polar coordinates
  /// \return sonar image output
  cv::Mat SonarImage() const;

  /// \brief Get the sonar mask for polar coordinater
  /// \return sonar mask
  cv::Mat SonarMask() const;

  /// \brief Set the near clip distance
  /// \param[in] _near near clip distance
  void SetNearClip(const double _near);

  /// \brief Set the far clip distance
  /// \param[in] _far far clip distance
  void SetFarClip(const double _far);


  /// \brief Set the vertical fov
  /// \param[in] _vfov vertical fov
  void SetVertFOV(const double _vfov);

  // Documentation inherited.
  virtual void RenderImpl();


  /**
   * @brief Documentation Iherited
   *
   * @param _type
   * @return true
   * @return false
   */
  virtual bool SetProjectionType(const std::string &_type);

  /**
   * @brief Pre render step
   *
   * @param _pose Set position of the camera
   */
  void PreRender(const math::Pose &_pose);

  /**
   * @brief Get the Sonar Image to cartesian cv::Mat
   *
   */
  void GetSonarImage();

  /**
   * @brief Get the Ros sonar msg
   *
   */
  uuv_sensor_ros_plugins_msgs::SonarStamped SonarRosMsg(const physics::WorldPtr _world);

  /**
   * @brief Update the data for the sonar
   *
   */
  void UpdateData();

  /**
   * @brief Cv mat to sonar bin data
   *
   * @param _accumData vector with all image data
   */
  void CvToSonarBin(std::vector<float> &_accumData);

  /**
   * @brief Get image width
   *
   * @return int Image width
   */
  int ImageWidth();

  /**
   * @brief Get image height
   *
   * @return int Image height
   */
  int ImageHeight();

  /**
   * @brief Get bin count
   *
   * @return int Bin count
   */
  int BinCount();

  /**
   * @brief Get beam count
   *
   * @return int Beam count
   */
  int BeamCount();


  /**
   * @brief Set the Bin Count object
   *
   * @param _value
   */
  void SetBinCount(const int &_value);

  /**
   * @brief Set the Beam Count object
   *
   * @param _value
   */
  void SetBeamCount(const int &_value);

  /**
   * @brief Set the Image Width object
   *
   * @param _value
   */
  void SetImageWidth(const int &_value);

  /**
   * @brief Set the Image Height object
   *
   * @param _value
   */
  void SetImageHeight(const int &_value);

  //// \brief Camera Texture for rendering
  Ogre::Texture* camTexture;

  //// \brief Camera Material
  Ogre::Material *camMaterial;

  //// \brief Camera Target
  Ogre::RenderTarget *camTarget;

  //// \brief Sonar beams depth data
  std::vector<int> sonarBinsDepth;


  //// \brief Bins vector
  std::vector<float> bins;

  // Sonar Image public variable
  cv::Mat sonarImage;


  /// \brief Update a render target.
  /// \param[in, out] _target Render target to update (render).
  /// \param[in, out] _material Material used during render.
  /// \param[in] _cam Camera to render from.
  /// \param[in] _updateTex True to update the textures in the material
private:
  void UpdateRenderTarget(Ogre::RenderTarget *_target,
                          Ogre::Material *_material,
                          Ogre::Camera *_cam,
                          const bool _updateTex = false);

  /// \brief Flag to check if the message was updated.
  bool bUpdated;

  //// \brief Sonar image from ogre
  Ogre::Image imgSonar;

  /********************** List of Debug Functions **********************/

  /**
   * @brief Print the vector to a matrix file with size beamCount per binCount
   *
   * @tparam T Content type of the matrix (float or int)
   * @param _filename FileName output
   * @param _matrix Input vector with matrix data
   */

  template <typename T> void DebugPrintMatrixToFile(const std::string &_filename, const std::vector<T> &_matrix);

  /**
   * @brief Print the image to a matrix file with size row x col
   *
   * @param _filename Filename output
   * @param _image Name of the output file
   */
  void DebugPrintImageToFile(const std::string &_filename, const cv::Mat &_image);

  /**
   * @brief Print the texture into a png file named "MyCamTest.png"
   *
   * @param _texture
   */
  void DebugPrintTexture(Ogre::Texture *_texture);

  /**
   * @brief Print a channel of Image file to a matrix colxrow
   *
   * @param _filename Name of the output file
   * @param _image Image to be written
   * @param _channel Channel selected
   */
  void DebugPrintImageChannelToFile(const std::string &_filename, const cv::Mat &_image, const int &_channel);

  /********************** End of Debug Functions List **********************/

  /**
   * @brief Print the results to a file
   *
   * @param _width
   * @param _height
   * @param _inTex
   */
protected:
  void ImageTextureToCV(float _width, int _height, Ogre::Texture* _inTex);


  /**
   * @brief Create transfer table from cartesian to polar
   *
   * @param _transfer Transfer vector that will be Generated
   */
  void GenerateTransferTable(std::vector<int> &_transfer);

  /**
   * @brief Transfer the sonar bin data to cv::Mat sonarImage using transfer matrix
   *
   * @param _accumData Vector with sonar bins data
   * @param _transfer Vector with tranfer function cartesian to polar
   */
  void TransferTableToSonar(const std::vector<float> &_accumData, const std::vector<int> &_transfer);

  /**
   * @brief
   *
   * @param _texture
   * @param _image
   * @param _width
   * @param _height
   */
  void PixelBoxTextureToCV(Ogre::Texture *_texture, cv::Mat &_image, int _width, int _height);




  /**
   * @brief Simple sigmoid function for bin intensity calculation
   *
   * @param x Input of sigmoid function
   * @return float Sigmoid value
   */
  float Sigmoid(float x);

  /// \brief Vertical field-of-view.
  double vfov;

  /// \brief Horizontal field-of-view.
  double hfov;


  /// \brief Near clip plane.
  double nearClip;

  /// \brief Far clip plane.
  double farClip;

  //// \brief Number of bins
  int binCount;

  //// \brief Image width for texture
  int imageWidth;

  //// \brief Image height for texture
  int imageHeight;


  //// \brief Image mask for polar image output
  cv::Mat sonarImageMask;

  //// \brief Image of the pure sonar image in cartesian coordinates

  //// \brief Raw image with texture data
  cv::Mat rawImage;

  //// \brief Number of beams
  int beamCount;

  //// \brief Data from sensor
  std::vector<float> accumData;


};
}  // namespace rendering
}  // namespace gazebo
#endif
