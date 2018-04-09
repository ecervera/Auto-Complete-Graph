#include "auto_complete_graph/Localization/ACG_localization.hpp"


g2o::VertexSE2RobotLocalization* AASS::acg::AutoCompleteGraphLocalization::addRobotLocalization(const g2o::SE2& se2, const Eigen::Affine3d& affine, const Eigen::Matrix3d& cov, const std::shared_ptr< perception_oru::NDTMap >& map){

	std::cout << "Adding the robot pose " << std::endl;
	g2o::VertexSE2RobotLocalization* robotlocalization =  new g2o::VertexSE2RobotLocalization();

	robotlocalization->setEstimate(se2);
	robotlocalization->setId(new_id_);
	++new_id_;

	robotlocalization->setMap(map);
	robotlocalization->setPose(affine);
	robotlocalization->setCovariance(cov);
	robotlocalization->initial_transfo = robotlocalization->estimate();

	_optimizable_graph.addVertex(robotlocalization);

// 	NDTNodeAndMap nodeAndMap(robot, map, affine);

	_nodes_localization.push_back(robotlocalization);
	return robotlocalization;
}
g2o::VertexSE2RobotLocalization* AASS::acg::AutoCompleteGraphLocalization::addRobotLocalization(const Eigen::Vector3d& rob_localization, const Eigen::Affine3d& affine, const Eigen::Matrix3d& cov,const std::shared_ptr< perception_oru::NDTMap >& map){
	g2o::SE2 se2(rob_localization(0), rob_localization(1), rob_localization(2));
	return addRobotLocalization(se2, affine, cov, map);
}
g2o::VertexSE2RobotLocalization* AASS::acg::AutoCompleteGraphLocalization::addRobotLocalization(double x, double y, double theta, const Eigen::Affine3d& affine, const Eigen::Matrix3d& cov,const std::shared_ptr< perception_oru::NDTMap >& map){
	Eigen::Vector3d robot1;
	robot1 << x, y, theta;
	return addRobotLocalization(robot1, affine, cov, map);
}





g2o::EdgeLocalization* AASS::acg::AutoCompleteGraphLocalization::addLocalization(double x, double y, double theta, int from_id, const Eigen::Matrix3d& information)
{
	g2o::SE2 se2(x, y, theta);
	return addLocalization(se2, from_id, information);

}

g2o::EdgeLocalization* AASS::acg::AutoCompleteGraphLocalization::addLocalization(const g2o::SE2& localization, int from_id, const Eigen::Matrix3d& information)
{
	
	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
	return addLocalization(localization, from_ptr, information);

}

g2o::EdgeLocalization* AASS::acg::AutoCompleteGraphLocalization::addLocalization(const g2o::SE2& localization, g2o::HyperGraph::Vertex* v1, const Eigen::Matrix3d& information)
{
	
	assert(information.isZero(1e-10) == false);
	
	g2o::EdgeLocalization* edge_loc = new g2o::EdgeLocalization;
	edge_loc->vertices()[0] = v1 ;
	edge_loc->vertices()[1] = _vertex_reference_for_montecarlo;
	edge_loc->setMeasurement(localization);
	edge_loc->setInformation(information);
	
// 	odometry->interface.setAge(_age_start_value);
	
	_optimizable_graph.addEdge(edge_loc);
	_edges_localization.push_back(edge_loc);
	return edge_loc;

}


g2o::VertexSE2Prior* AASS::acg::AutoCompleteGraphLocalization::setPriorReference()
{
	if(_nodes_prior.size() > 0) {
		_vertex_reference_for_montecarlo = _nodes_prior[0];
		return _vertex_reference_for_montecarlo;
	}
	return NULL;
}


void AASS::acg::AutoCompleteGraphLocalization::updateNDTGraph(const auto_complete_graph::GraphMapLocalizationMsg& ndt_graph_localization) {

	addNDTGraph(ndt_graph_localization);

//	updateLocalizationEdges(ndt_graph_localization);


}

void AASS::acg::AutoCompleteGraphLocalization::addNDTGraph(const auto_complete_graph::GraphMapLocalizationMsg& ndt_graph_localization)
{
	
	std::cout << "Node in graph " << ndt_graph_localization.graph_map.nodes.size() << std::endl;
	
	if(_previous_number_of_node_in_ndtgraph != ndt_graph_localization.graph_map.nodes.size() ){
		
// 		auto factors = ndt_graph.factors();
		
		size_t i;
		i = _previous_number_of_node_in_ndtgraph - 1;
		
		assert(i >= 0);
		
		for (i; i < ndt_graph_localization.graph_map.nodes.size() - 1 ; ++i) {

			g2o::VertexSE2RobotPose* robot_ptr;
			g2o::VertexSE2RobotLocalization* robot_localization_ptr;
			std::shared_ptr<perception_oru::NDTMap> map;
//			g2o::SE2 robot_pos;
			std::tie(robot_ptr, robot_localization_ptr, map) = addElementNDT(ndt_graph_localization, i);
			assert(robot_ptr != NULL);
			std::cout << "TEST pointer " << std::endl; std::cout << robot_ptr->getPose().matrix() << std::endl;
			//********************** Extract the corners *****************//
			if(_extract_corners) {
				extractCornerNDTMap(map, robot_ptr, robot_localization_ptr);
			}
			//********************** Add the time stamp ******************//
// 			robot_ptr->setTime(ndt_graph.nodes[i].time_last_update);
			
		}
		
		_previous_number_of_node_in_ndtgraph = ndt_graph_localization.graph_map.nodes.size();
		
	}

	updateLinks();

	std::cout << "Test no double links" << std::endl;
	noDoubleLinks();

	std::cout << "After " << std::endl;
	
	

}

std::tuple<g2o::VertexSE2RobotPose*, g2o::VertexSE2RobotLocalization*, std::shared_ptr<perception_oru::NDTMap> > AASS::acg::AutoCompleteGraphLocalization::addElementNDT(const auto_complete_graph::GraphMapLocalizationMsg& ndt_graph_localization, int element)
{
	///ATTENTION Indexes start at 1 instead of 0 :/
	std::cout << "good indexes " << ndt_graph_localization.graph_map.nodes[element].id.data << " = " << element << " voilaaa " << std::endl;
	assert(ndt_graph_localization.graph_map.nodes[element].id.data == element);
	//Return Gaussian white noise
// 	auto randomNoise = [](double mean, double deviationt) -> double {
// 		std::default_random_engine engine{std::random_device()() };
// 		std::normal_distribution<double> dist(mean, deviationt);
// 		return dist(engine);
// 	};
	
	Eigen::Vector3d diff_vec = getLastTransformation();
	
	//Calculate noise

	std::cout << "Checking node nb " << element << std::endl;
	
	//RObot pose
// 			ndt_feature::NDTFeatureNode& feature = dynamic_cast<ndt_feature::NDTFeatureNode&>(ndt_graph.getNodeInterface(i));
	std::cout << "Copy feature" << std::endl;
	
	auto geometry_pose = ndt_graph_localization.graph_map.nodes[element].pose;
	Eigen::Affine3d affine;
	tf::poseMsgToEigen(geometry_pose, affine);
// 			feature.copyNDTFeatureNode( (const ndt_feature::NDTFeatureNode&)ndt_graph.getNodeInterface(i) );
// 	Eigen::Affine3d affine = Eigen::Affine3d(ndt_graph.getNodeInterface(element).getPose());
	
	Eigen::Isometry2d isometry2d = Affine3d2Isometry2d(affine);
	
	//TODO make this work
// 			isometry2d =  diff * isometry2d;
	
	//BUG IS AFTER
	std::cout << "print" << std::endl;
	
	auto robot_pos = g2o::SE2(isometry2d);
// 			std::cout << "robot pose done : " << isometry2d.matrix() << std::endl;
	g2o::SE2 diff_vec_se2(diff_vec);
// 			std::cout << "diff vec done" << diff_vec << std::endl;

	///WARNING trying to change this to fit GraphMap model. Node are at the exact pose so no need to translate them AND factor I don't know yet
	robot_pos = robot_pos * diff_vec_se2;
	std::cout << "multiply" << std::endl;
		
	auto map_msg =  ndt_graph_localization.graph_map.nodes[element].ndt_map;

	perception_oru::NDTMap* map;
	perception_oru::NDTMap* map_copy;
	perception_oru::LazyGrid* lz;
	std::string frame;
	bool good2 = perception_oru::fromMessage(lz, map_copy, map_msg, frame);
	std::shared_ptr<perception_oru::NDTMap> shared_map(map_copy);



	assert(shared_map->getAllInitializedCells().size() != 0);
	assert(shared_map->getAllInitializedCells().size() == map_msg.cells.size());
		
// 	std::cout << "get res" << std::endl;
// // 			double resolution = dynamic_cast<ndt_feature::NDTFeatureNode&>( ndt_graph.getNodeInterface(i) ).map->params_.resolution;
// // 			Use a a msg to copy to a new pointer so it doesn't get forgotten :|
// 	ndt_map::NDTMapMsg msg;
// // 			ATTENTION Frame shouldn't be fixed
// 	bool good = perception_oru::toMessage(map, msg, "/world");
// 	perception_oru::NDTMap* map_copy;
// 	perception_oru::LazyGrid* lz;
// 	std::string frame;
// 	bool good2 = perception_oru::fromMessage(lz, map_copy, msg, frame);
// 	std::shared_ptr<perception_oru::NDTMap> shared_map(map_copy);

	auto robot_ptr = addRobotPose(robot_pos, affine, shared_map);

	robot_ptr->setIndexGraphMap(ndt_graph_localization.graph_map.nodes[element].id.data);

	assert(robot_ptr != NULL);
	//Add Odometry if it is not the first node
	
	if(element > 0 ){

		if(_do_own_registration) {
			//From my understanding, for now factors are othe same as all the registrations before. Hence I make it a big registration between the submaps ;)

			auto from = _nodes_ndt[ndt_graph_localization.graph_map.factors[element - 1].prev.data];
			auto toward = _nodes_ndt[ndt_graph_localization.graph_map.factors[element - 1].next.data];
			g2o::SE2 from_vec = from->estimate();
			g2o::SE2 toward_vec = toward->estimate();
			g2o::SE2 odom = from_vec.inverse() * toward_vec;

			std::cout << "From " << from_vec.toVector() << " toward " << toward_vec.toVector() << " thus odom " << odom.toVector() << std::endl;
			std::cout << "Make sure that this is correct before moving forward" << std::endl;
			int waitwait;
			std::cin >> waitwait;


			Eigen::Affine3d odom_affine = se2ToAffine3d(odom);

			Eigen::Affine3d odom_register;
			Eigen::MatrixXd odom_cov;
			std::tie(odom_register, odom_cov) = registerSubmaps(*from, *toward, odom_affine, 2);

			Eigen::Isometry2d isometry2d_odometry = Affine3d2Isometry2d(odom_register);
			g2o::SE2 odometry(isometry2d_odometry);

			std::cout << "Saving cov to 2d" << std::endl;
			Eigen::Matrix3d cov_2d;
			cov_2d << odom_cov(0, 0), odom_cov(0, 1), 0,
					odom_cov(1, 0), odom_cov(1, 1), 0,
					0, 0, odom_cov(5, 5);

// 		tf::matrixEigenToMsg(cov, ndt_graph.factors[element - 1].covariance);
			std::cout << "Saving information " << std::endl;
			Eigen::Matrix3d information = cov_2d.inverse();

			std::cout << "Saving odometry " << odometry.toVector() << " from " << from << " toward " << toward << " info " << information << " " << std::endl;
			addOdometry(odometry, from, toward, information);

		}
		else {

			std::cout << "adding the odometry" << std::endl;

			//The factors are expressed as transformation between the node expressed in the world frame. They need to be translated to the robot frame I believe?
			auto geometry_pose_factor = ndt_graph_localization.graph_map.factors[element - 1].diff;
			Eigen::Affine3d affine_factor;
			tf::poseMsgToEigen(geometry_pose_factor, affine_factor);
			Eigen::Isometry2d isometry2d_odometry = Affine3d2Isometry2d(affine_factor);

			////////OLD VERSION
// 		double x = cumulated_translation(0, 3);
// 		double y = cumulated_translation(1, 3);
			g2o::SE2 odometry(isometry2d_odometry);

			////////NEW VERSION
			//We need to transform the odometry into he robot pose coordinate frame see https://en.wikipedia.org/wiki/Change_of_basis#Change_of_coordinates_of_a_vector

			//Find the matrix describing the rotation and left multiply the odometry by the inverse of it.
//		double angle = robot_pos.toVector()[2];
//		Eigen::Matrix2d rot;
//		rot << std::cos(angle), - std::sin(angle),
//			   std::sin(angle),   std::cos(angle);
//		Eigen::Vector2d odom_2d_translation = isometry2d_odometry.translation();
//		Eigen::Vector2d odom_2d_translation_robot_frame = rot.inverse() * odom_2d_translation;
//		Eigen::Vector3d odom_3d_translation_robot_frame;
//		odom_3d_translation_robot_frame << odom_2d_translation_robot_frame[0], odom_2d_translation_robot_frame[1], 0;
//		g2o::SE2 odometry(odom_3d_translation_robot_frame);
//
//		std::cout << "Odom " << odom_2d_translation_robot_frame[0] << " " << odom_2d_translation_robot_frame[1] << std::endl;

			std::cout << "ODOM " << odometry.toVector() << std::endl;
//		exit(0);

// 		g2o::SE2 odometry = NDTFeatureLink2EdgeSE2(links[element - 1]);

			std::cout << " ref " << ndt_graph_localization.graph_map.factors[element - 1].prev.data << " and mov "
			          << ndt_graph_localization.graph_map.factors[element - 1].next.data << " ndt node size "
			          << _nodes_ndt.size() << std::endl;

			///ATTENTION Indexes of graph_map start at 1 instead of 0 :/
			assert(ndt_graph_localization.graph_map.factors[element - 1].prev.data < _nodes_ndt.size());
			assert(ndt_graph_localization.graph_map.factors[element - 1].next.data < _nodes_ndt.size());
			assert(ndt_graph_localization.graph_map.factors[element - 1].prev.data >= 0);
			assert(ndt_graph_localization.graph_map.factors[element - 1].next.data >= 0);

			auto from = _nodes_ndt[ndt_graph_localization.graph_map.factors[element - 1].prev.data];
			auto toward = _nodes_ndt[ndt_graph_localization.graph_map.factors[element - 1].next.data];

			assert(from->getIndexGraphMap() == ndt_graph_localization.graph_map.factors[element - 1].prev.data);
			assert(toward->getIndexGraphMap() == ndt_graph_localization.graph_map.factors[element - 1].next.data);

			auto from_vec = from->estimate().toVector();
			auto toward_vec = toward->estimate().toVector();
			g2o::SE2 odom_tmp = toward->estimate() * (from->estimate()).inverse();
			std::cout << "TRansof" << from_vec << " toward " << toward_vec << " is " << toward_vec - from_vec
			          << std::endl;
			std::cout << "But the composition is " << (odometry * from->estimate()).toVector() << std::endl;
			std::cout << "But the composition is " << (odom_tmp * from->estimate()).toVector() << std::endl;
			int wait = 0;
			std::cin >> wait;


			std::cout << "Saving cov " << std::endl;
			//TODO : transpose to 3d and use in odometry!

			auto cov_msg = ndt_graph_localization.graph_map.factors[element - 1].covariance;

			std::vector<double>::const_iterator it;
			it = cov_msg.data.begin();
			std::cout << "Cov size " << cov_msg.data.size() << std::endl;
			assert(cov_msg.data.size() == 36); //6x6 matrix

			std::vector<double>::const_iterator it_2;
			it_2 = cov_msg.data.begin();
			std::cout << cov_msg.data.size() << std::endl;
			assert(cov_msg.data.size() == 36);

			Eigen::MatrixXd cov_3d(6, 6);
			for (size_t i = 0; i < 6; ++i) {
				for (size_t j = 0; j < 6; ++j) {
					cov_3d(i, j) = cov_msg.data[(6 * i) + j];
				}
			}

			std::cout << "Saving cov to 2d" << std::endl;
			Eigen::Matrix3d cov_2d;
			cov_2d << cov_3d(0, 0), cov_3d(0, 1), 0,
					cov_3d(1, 0), cov_3d(1, 1), 0,
					0, 0, cov_3d(5, 5);

// 		tf::matrixEigenToMsg(cov, ndt_graph.factors[element - 1].covariance);
			std::cout << "Saving information " << std::endl;
			Eigen::Matrix3d information = cov_2d.inverse();

// 				if(noise_flag = true && i != 0){
// 					odometry = odometry * noise_se2;
// 				}

			std::cout << "Saving odometry " << odometry.toVector() << " from " << from << " toward " << toward
			          << " info " << information << " " << std::endl;
			addOdometry(odometry, from, toward, information);
			std::cout << ">Done" << std::endl;
		}
	}

	auto robot_localization_ptr = addLocalizationEdges(ndt_graph_localization, element, shared_map);
	std::tuple<g2o::VertexSE2RobotPose*, g2o::VertexSE2RobotLocalization*, std::shared_ptr<perception_oru::NDTMap> > tuple(robot_ptr, robot_localization_ptr, shared_map);
	return tuple;

}

g2o::VertexSE2RobotLocalization* AASS::acg::AutoCompleteGraphLocalization::addLocalizationEdges( const auto_complete_graph::GraphMapLocalizationMsg &ndt_graph_localization, int element, const std::shared_ptr<perception_oru::NDTMap>& shared_map) {

	auto localization_msg = ndt_graph_localization.localizations[element];
	AASS::acg::Localization localization;
	AASS::acg::fromMessage(localization_msg, localization);
	std::cout << "Reading MCL Localization position : " << localization.mean[0] << ", " << localization.mean[1] << ", " << localization.mean[2] << std::endl;
	g2o::SE2 se2loc(localization.mean[0], localization.mean[1], localization.mean[2]);
	Eigen::Matrix3d cov = localization.cov;

	auto affine3d = se2ToAffine3d(se2loc, _z_elevation);
	return addRobotLocalization(se2loc, affine3d, cov, shared_map);
	//No localization for now
//	addLocalization(se2loc, robot_ptr, localization.cov.inverse());

}



//TODO: refactor this!
void AASS::acg::AutoCompleteGraphLocalization::extractCornerNDTMap(const std::shared_ptr<perception_oru::NDTMap>& map, g2o::VertexSE2RobotPose* robot_ptr, g2o::VertexSE2RobotLocalization* robot_localization)
{

	int corners_added = 0, observations_added = 0;

	//HACK For now : we translate the Corner extracted and not the ndt-maps
	auto cells = map->getAllCellsShared();
	std::cout << "got all cell shared " <<  cells.size() << std::endl;
	double x2, y2, z2;
	map->getCellSizeInMeters(x2, y2, z2);
	std::cout << "got all cell sized" << std::endl;
	double cell_size = x2;

	std::vector<AASS::acg::AutoCompleteGraph::NDTCornerGraphElement> corners_end;
	getAllCornersNDTTranslatedToGlobalAndRobotFrame(map, robot_ptr, corners_end);

	/***************** ADD THE CORNERS INTO THE GRAPH***********************/

	Eigen::Matrix3d cov = robot_localization->getCovariance();
	//Remove the rotationnal component
	Eigen::Matrix2d cov_2d;
	cov_2d << cov(0,0), cov(0,1),
			cov(1,0), cov(1,1);

	//If corners are found too close we fuse them:
	std::set<g2o::VertexLandmarkNDT*> corner_seen_here;

	for(size_t i = 0 ; i < corners_end.size() ; ++i){
		corners_added++;
// 		std::cout << "checking corner : " << _nodes_landmark.size() << std::endl ;  /*corners_end[i].print()*/ ; std::cout << std::endl;
		bool seen = false;
		g2o::VertexLandmarkNDT* ptr_landmark_seen = NULL;
		for(size_t j = 0 ; j <_nodes_landmark.size() ; ++j){
// 			if(tmp[j] == _corners_position[i]){
			g2o::Vector2 landmark = _nodes_landmark[j]->estimate();
			cv::Point2f point_land(landmark(0), landmark(1));

			double res = cv::norm(point_land - corners_end[i].position);

//			std::cout << "res : " << std::endl;

			//If we found the landmark, we save the data
			if( res < cell_size * 2){
				seen = true;
				ptr_landmark_seen = _nodes_landmark[j];
			}
		}

		auto ptr_find = corner_seen_here.find(ptr_landmark_seen);
		//If the corner was not added this turn around
		if(ptr_find == corner_seen_here.end()) {
			if (seen == false) {
//				std::cout << "New point " << i << std::endl;
// 			assert(i < ret_export.size());
				g2o::Vector2 position_globalframe;
				position_globalframe << corners_end[i].position.x, corners_end[i].position.y;
// 			g2o::VertexLandmarkNDT* ptr = addLandmarkPose(vec, ret_export[i].getMeanOpenCV(), 1);

				cv::Point2f p_observation;
//				std::cout << "New point " << i << std::endl;
				p_observation.x = corners_end[i].getObservations()(0);
//				std::cout << "New point " << i << std::endl;
				p_observation.y = corners_end[i].getObservations()(1);
//				std::cout << "New point " << i << std::endl;
				g2o::VertexLandmarkNDT *ptr = addLandmarkPose(position_globalframe, p_observation, 1);
				ptr->addAnglesOrientations(corners_end[i].getAngleWidths(), corners_end[i].getOrientations());
				ptr->first_seen_from = robot_ptr;

				//TESTING to visualise which cells gave the corner
				ptr->cells_that_gave_it_1 = corners_end[i].cells1;
				ptr->cells_that_gave_it_2 = corners_end[i].cells2;
				ptr->robotpose_seen_from = robot_ptr->estimate();
				//END OF TEST

				corner_seen_here.insert(ptr);
				std::cout << "Adding corner " << ptr << " from " << corners_end[i].getNodeLinkedPtr() << " and " << robot_localization << std::endl;

				addLandmarkObservation(corners_end[i].getObservations(), corners_end[i].getNodeLinkedPtr(), ptr);
				//Add covariance given by MCL instead of the default one.
				addLandmarkObservation(corners_end[i].getObservations(), robot_localization, ptr, cov_2d);

				observations_added++;
				observations_added++;


			} else {
				std::cout << "Point seen: " << ptr_landmark_seen << " from " << corners_end[i].getNodeLinkedPtr() << " and " << robot_localization << std::endl;
				addLandmarkObservation(corners_end[i].getObservations(), corners_end[i].getNodeLinkedPtr(),
				                       ptr_landmark_seen);
				//Add covariance given by MCL instead of the default one.
				addLandmarkObservation(corners_end[i].getObservations(), robot_localization, ptr_landmark_seen, cov_2d);
				observations_added++;
				observations_added++;

				corner_seen_here.insert(ptr_landmark_seen);
			}
		}
		else{
			std::cout << "Created in this loop" << std::endl;
		}
	}
//	assert(corners_added * 2 == observations_added);

}


void AASS::acg::AutoCompleteGraphLocalization::getAllCornersNDTTranslatedToGlobalAndRobotFrame(const std::shared_ptr<perception_oru::NDTMap>& map, g2o::VertexSE2RobotPose* robot_ptr, std::vector<AASS::acg::AutoCompleteGraph::NDTCornerGraphElement>& corners_end)
{

	perception_oru::ndt_feature_finder::NDTCorner cornersExtractor;
	std::cout << "hopidy" << std::endl;
	auto ret_export = cornersExtractor.getAllCorners(*map);
	std::cout << "gotall corner" << std::endl;
// 	auto ret_opencv_point_corner = cornersExtractor.getAccurateCvCorners();
	std::cout << "got all accurate corners" << std::endl;
// 	auto angles = cornersExtractor.getAngles();
// 	std::cout << "got all angles" << std::endl;

	auto it = ret_export.begin();

	std::cout << "Found " << ret_export.size() << " corners " << std::endl;
	//Find all the observations :

	//**************** HACK: translate the corners now : **************//

// 	int count_tmp = 0;
	for(it ; it != ret_export.end() ; ++it){
		std::cout << "Corner size " << it->getOrientations().size() << std::endl;
		//Limited to corners that possess an orientation.
		if(it->getOrientations().size() > 0){
			Eigen::Vector3d vec;
			// 		vec << it->x, it->y, angles[count_tmp].second;
			std::cout << "Corner size " << std::endl;
			vec << it->getMeanOpenCV().x, it->getMeanOpenCV().y, it->getOrientations()[0];

			std::cout << "Corner size " << std::endl;
			cv::Point2f p_out;
			Eigen::Vector3d landmark_robotframe;
			translateFromRobotFrameToGlobalFrame(vec, robot_ptr->estimate(), landmark_robotframe);

			std::cout << "Corner size " << std::endl;
			p_out.x = landmark_robotframe(0);
			p_out.y = landmark_robotframe(1);

			Eigen::Vector2d observation; observation << vec(0), vec(1);
			std::vector<double> orientations;
			std::vector<double> angles_width;

			std::cout << "Corner size " << std::endl;
			double angle_landmark = vec(2);

			std::cout << "Corner size " << std::endl;
			for(auto it_orientation = it->getOrientations().begin() ; it_orientation != it->getOrientations().end() ; ++it_orientation){
				std::cout << "Pushing back orientation" << std::endl;
				orientations.push_back((*it_orientation));
			}
			for(auto it_angles = it->getAngles().begin() ; it_angles != it->getAngles().end() ; ++it_angles){
				std::cout << "Pushing back angle" << std::endl;
				angles_width.push_back((*it_angles));
			}



			//***************************Old version for testing***************************//
			// 		Eigen::Vector2d trueObservation2d = robot_pos.inverse() * real_obs;
			// 		std::cout << trueObservation(0) << " == " << trueObservation2d(0) << std::endl;
			// 		assert(trueObservation(0) == trueObservation2d(0));
			// 		std::cout << trueObservation(1) << " == " << trueObservation2d(1) << std::endl;
			// 		assert(trueObservation(1) == trueObservation2d(1));
			// 		observation2d_test = trueObservation2d;
			// 		std::cout << observation(0) << " == " << observation2d_test(0) << " minus " << observation(0) - observation2d_test(0) << std::endl;
			// 		assert(trueObservation(0) == trueObservation2d(0));
			// 		std::cout << observation(1) << " == " << observation2d_test(1) << " minus " << observation(1) - observation2d_test(1) << std::endl;
			// 		int a ;
			// 		std::cin >> a;
			//******************************************************************************//



			// 		std::cout << "Node transfo " << ndt_graph.getNode(i).T.matrix() << std::endl;
			std::cout << "Position node " << robot_ptr->estimate().toVector() << std::endl;
			std::cout << " vec " << vec << std::endl;
			// 		std::cout << "Well " << robot_pos.toVector() + vec << "==" << ndt_graph.getNode(i).T * vec << std::endl;

			//ATTENTION THIS IS NOT TRUE BUT REALLY CLOSE
			// 				assert (robot_pos + vec == ndt_graph.getNode(i).T * vec);

			std::cout << "NEW POINT : "<< p_out << std::endl;

			NDTCornerGraphElement cor(p_out);
			cor.addAllObserv(robot_ptr, observation, orientations, angles_width);
			cor.cells1 = it->getCells1();
			cor.cells2 = it->getCells2();
			corners_end.push_back(cor);
			// 		count_tmp++;
		}

	}

}


void AASS::acg::AutoCompleteGraphLocalization::testInfoNonNul(const std::string& before) const {

	std::cout << "Test info non nul after " << before << std::endl;
	auto idmapedges = _optimizable_graph.edges();

	for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
		assert((*ite)->vertices().size() >= 1);

		g2o::EdgeLandmark_malcolm* ptr = dynamic_cast<g2o::EdgeLandmark_malcolm*>(*ite);
		g2o::EdgeSE2Prior_malcolm* ptr1 = dynamic_cast<g2o::EdgeSE2Prior_malcolm*>(*ite);
		g2o::EdgeOdometry_malcolm* ptr2 = dynamic_cast<g2o::EdgeOdometry_malcolm*>(*ite);
		g2o::EdgeLinkXY_malcolm* ptr3 = dynamic_cast<g2o::EdgeLinkXY_malcolm*>(*ite);
//		g2o::EdgeLocalization* ptr4 = dynamic_cast<g2o::EdgeLocalization*>(*ite);
		if(ptr != NULL){
			assert(ptr->information().isZero(1e-10) == false);
		}
		else if(ptr1 != NULL){
			assert(ptr1->information().isZero(1e-10) == false);
		}
		else if(ptr2 != NULL){
			assert(ptr2->information().isZero(1e-10) == false);
		}
		else if(ptr3 != NULL){
			assert(ptr3->information().isZero(1e-10) == false);
		}
//		else if(ptr4 != NULL){
//			assert(ptr4->information().isZero(1e-10) == false);
//		}
		else{
			throw std::runtime_error("Didn't find the type of the edge :S");
		}
	}


}



std::tuple<Eigen::Affine3d, Eigen::MatrixXd> AASS::acg::AutoCompleteGraphLocalization::registerSubmaps(const g2o::VertexSE2RobotPose& from, const g2o::VertexSE2RobotPose& toward, Eigen::Affine3d &transformation,	int nb_neighbor) {
//	void ndt_feature::NDTFeatureGraph::registerSubmaps(NDTFeatureLink &link, int nb_neighbours, bool keepScore) {
	perception_oru::NDTMatcherD2D matcher_d2d;
	matcher_d2d.n_neighbours = nb_neighbor;

	std::cout << "Matching : " << from.getIndexGraphMap() << " with " << toward.getIndexGraphMap() << std::endl;
//		Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
	std::cout << "Transform between the two " <<transformation.matrix() << std::endl;
	int a ;
// 	std::cout << "PAUSE before match" << std::endl;
// 	std::cin >> a;

	Eigen::Affine3d before_T = transformation;

	bool converged = matcher_d2d.match(*(from.getMap().get()), *(toward.getMap().get()), transformation, true);

	std::cout << "Transform between the two new " <<transformation.matrix() << std::endl;

// 	int a ;
// 	std::cout << "PAUSE before cov" << std::endl;
// 	std::cin >> a;

	//Adding the covariance into the link
	Eigen::MatrixXd cov(6,6);

	bool same = true;
	for(size_t i = 0; i < 4 ; ++i){

		for(size_t  j = 0 ; j < 4 ; ++j){
			if(before_T(i,j) != transformation(i,j)){
				same = false;
			}
		}
	}

	if(!same){
		cov.setZero();
		matcher_d2d.covariance(*(from.getMap().get()), *(toward.getMap().get()), transformation, cov);
	}
	else{
		std::cout << "NOTHING HAPPENED Creating a identity matrix" << std::endl;
		cov = Eigen::MatrixXd::Identity(6, 6);
		cov << 	0.02, 	0, 		0,    0,    0,    0,
				0, 		0.02, 	0,    0,    0,    0,
				0, 		0, 		0.02, 0,    0,    0,
				0, 		0, 		0,	  0.02, 0,    0,
				0, 		0,	 	0,	  0,    0.02, 0,
				0, 		0, 		0,	  0,    0,    0.02;
// 		exit(0);
	}
	std::cout << "Size of Covariance : " << cov.rows() << " AND COLS " << cov.cols() << std::endl;

	assert(cov.rows() == 6);
	assert(cov.cols() == 6);
	std::cout << "PAUSE got cov : " << cov << "\n";
	std::cout << "COVARIANCE BY MATCHER " << cov.inverse() << "\n";

	if(!converged){
// 		throw std::runtime_error("ndt_map registration didn't converge");
		std::cout << "USing odometry input a number to continue" << std::endl;
		int a;
		std::cin >> a;
	}
	if(same){
		std::cout << "Manually created the covariance because the matching returned the same transof as before." << std::endl;
		int a;
		std::cin >> a;
	}
// 	exit(0);
// 	std::cin >> a;

	std::cout << "End of link" << std::endl;

	return std::tuple<Eigen::Affine3d, Eigen::MatrixXd>(transformation, cov);
}
