#include "Robot.hpp"
#include <iostream>
#include <smurf/Smurf.hpp>
#include <envire_core/items/Transform.hpp>
#include <base/Time.hpp>
#include <envire_core/items/Item.hpp>
#include <configmaps/ConfigData.h>

void envire::envire_smurf::Robot::welcome()
{
    std::cout << "You successfully compiled and executed the envire_smurf Project. Welcome!" << std::endl;
}

void envire::envire_smurf::Robot::loadFromSmurf(envire::core::TransformGraph &graph, const std::string& path)
{
  // We want to be able to verify that every frame defined is connected through
  //some tranformation. That's why all the frames are included first
    smurf::Robot robot;
    robot.loadFromSmurf(path);
    // Frames
    std::vector<smurf::Frame *> frames= robot.getFrames();
    std::cout << "Iterate over the frames:" << std::endl;
    for(std::vector<smurf::Frame *>::iterator it = frames.begin(); it != frames.end(); ++it)
    {
//        base::Time time = base::Time::now();
//        base::TransformWithCovariance tf_cov;
//        envire::core::Transform envire_tf(time, tf_cov);
//        std::cout << "Include the following frame in the graph: " << (*it)->getName() << std::endl;
        // Make sure this method works 
        //graph.addFrame(frame->getName());
        //
        // By now we need: objects in that frame and type: link, joint (with
        // type) and sensor. This information goes in the config map of the
        // envire item
        //
        // Add an Item ConfigMap with this information to the frame
        //
        // Fill configMap with the node information and add it to the frame

        std::string frame_id = (*it)->getName();
        graph.addFrame(frame_id);

        std::pair <std::string ,  smurf::Frame* >  link;
        link.first="LINK";
        //link.second=*(*it);
        link.second=(*it);
        boost::shared_ptr<envire::core::Item<std::pair <std::string , smurf::Frame *  >  > >link_itemPtr (new  envire::core::Item<std::pair <std::string ,smurf::Frame *  >  > );
        link_itemPtr->setData(link);
        graph.addItemToFrame(frame_id, link_itemPtr);

/*
//////////////////////////////////////////////adding smurf collisions//////////////////////////////////////
        std::pair <std::string ,  std::vector<smurf::Collidable> >  smurf_collidable;
        smurf_collidable.first="SMURF::COLLIDABLE";
        std::vector<smurf::Collidable> frame_collisons= (*it)->getCollisionObjects();
        smurf_collidable.second=frame_collisons;
        boost::shared_ptr<envire::core::Item<std::pair <std::string ,  std::vector<smurf::Collidable> >  > >collisons_itemPtr (new  envire::core::Item<std::pair <std::string ,  std::vector<smurf::Collidable> >  > );
        collisons_itemPtr-> setData(smurf_collidable);
        graph.addItemToFrame(frame_id, collisons_itemPtr);

//////////////////////////////////////////////adding smurf visuals//////////////////////////////////////
        std::pair <std::string ,  std::vector<smurf::Visual> >  smurf_visual;
        smurf_visual.first="SMURF::VISUAL";
        std::vector<smurf::Visual> frame_visuals= (*it)->getVisuals();
        smurf_visual.second=frame_visuals;
        boost::shared_ptr<envire::core::Item<  std::pair <std::string ,  std::vector<smurf::Visual> >    > >visuals_itemPtr (new  envire::core::Item< std::pair <std::string ,  std::vector<smurf::Visual> > > );
        visuals_itemPtr-> setData(smurf_visual);
        graph.addItemToFrame(frame_id, visuals_itemPtr);
*/
    }

//////////////////////////////////////////////////////////////adding sensors///////////////////////////////////////////////////////////////////////////

    std::vector<smurf::Sensor *> robot_Sensors= robot.getSensors();
    std::cout << "Iterate over the  Sensors: " << std::endl;
    std::string frame_id;
    for(std::vector<smurf::Sensor *>::iterator it = robot_Sensors.begin(); it != robot_Sensors.end(); ++it)
    {

        frame_id=(*it)->getattachmentPoint()->getName();
        std::pair <std::string ,  smurf::Sensor* >  smurf_sensor;
        smurf_sensor.first="SENSOR";
        //smurf_sensor.second=*(*it);
        smurf_sensor.second=(*it);
        boost::shared_ptr<envire::core::Item<std::pair< std::string , smurf::Sensor* > > > sensor_itemPtr (new  envire::core::Item< std::pair <std::string ,  smurf::Sensor* > > );
        sensor_itemPtr-> setData(smurf_sensor);
        graph.addItemToFrame(frame_id, sensor_itemPtr);
    }

    // Static Transformations: All transformations are considered static initially
    std::vector<smurf::StaticTransformation *> staticTfs= robot.getStaticTransforms();
    std::cout << " Static transformations " << std::endl;
    for(std::vector<smurf::StaticTransformation *>::iterator it = staticTfs.begin(); it != staticTfs.end(); ++it) {
        smurf::Frame source = (*it) -> getSourceFrame();
        envire::core::FrameId sourceId = source.getName();
        smurf::Frame target = (*it) -> getTargetFrame();
        envire::core::FrameId targetId = target.getName();
        Eigen::Affine3d tf_smurf = (*it) -> getTransformation();
        std::cout << "Transformation from " << sourceId <<" to " << targetId << " is " << tf_smurf.matrix() << std::endl;
        base::Time time = base::Time::now();
        base::TransformWithCovariance tf_cov(tf_smurf);
        envire::core::Transform envire_tf(time, tf_cov);
        graph.addTransform(sourceId, targetId, envire_tf);

        std::pair <std::string ,  smurf::StaticTransformation* >  jonit;
        jonit.first="JOINT";
        jonit.second=(*it);
        boost::shared_ptr<envire::core::Item<std::pair< std::string , smurf::StaticTransformation* > > > joint_itemPtr (new  envire::core::Item< std::pair <std::string ,  smurf::StaticTransformation* > > );
        joint_itemPtr->setData(jonit);
        graph.addItemToFrame(sourceId,joint_itemPtr);
    }


}
