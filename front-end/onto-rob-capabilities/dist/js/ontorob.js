angular.module('ontoRobApp', ['ui.bootstrap','ui.router'])
.config(function($urlRouterProvider,$locationProvider,$stateProvider){
	$urlRouterProvider.otherwise("/"); // 404
	$locationProvider.html5Mode(true);
	
	$stateProvider
		.state(
			"index",
			{
				url:"/",
				templateUrl:"home.html",
				controller:"indexCtrl"
			}
		)
		.state(
			"capabilities-ui",
			{
				url:"/capabilities-ui",
				templateUrl:"capabilities-ui.html",
				controller:"ontorobCtrl"
			}
		)
})
.controller('ontorobCtrl', ['$scope', '$http','$state','$compile','$interval','Data', ontorobCtrl])
.controller('indexCtrl', ['$scope', '$http','$timeout','$window','$state','Data', indexCtrl])
.service("Data", dataService);

function dataService () {
	this.capabilities = [];
	this.serverPort = 5000;
	this.streamPort = 8080;
	
	// gianluca's laptop on eduroam
	// this.ip = "http://137.108.118.237:"+this.serverPort+"/";
	// this.streamIp = "http://137.108.118.237:"+this.streamPort+"/";
	
	// bender on eduroam
	this.ip = "http://137.108.125.182:"+this.serverPort+"/";
	this.streamIp = "http://137.108.125.182:"+this.streamPort+"/";
	
	// ardrone wifi
	// this.ip = "http://192.168.1.3:"+this.serverPort+"/";
// 	this.streamIp = "http://192.168.1.3:"+this.streamPort+"/";

}

function indexCtrl($scope,$http,$timeout,$window,$state, Data) {
	$scope.errorUrl = "error.html";
	$scope.successUrl = "capabilities-ui.html";

	// TODO automatically filtered topics
	// this should be done better automatically
	$scope.topicToFilter = [
		"/move_base/global_costmap/obstacle_layer/clearing_endpoints",
		"/move_base/local_costmap/obstacle_layer/clearing_endpoints",
		"/move_base/current_goal",
		"/cmd_vel_mux/input/safety_controller",
		"/cmd_vel_mux/input/navi",
		"/cmd_vel_mux/input/switch",
		"/navigation_velocity_smoother/raw_cmd_vel",
		"/cmd_vel_mux/input/teleop",
		"/move_base/current_goal",
		"/camera/depth/image_raw",
		"/camera/rgb/image_raw/compressed",
		"/move_base/local_costmap/costmap",
		"/move_base/global_costmap/costmap",
		"/camera/rgb/image_raw/compressedDepth",
		"/joint_states",
		"/ardrone/front/image_raw/compressed",
		"/ardrone/image_raw/compressedDepth",
		"/ardrone/front/image_raw/compressedDepth",
		"/ardrone/bottom/image_raw/compressedDepth",
		"/ardrone/image_raw/compressed",
		"/ardrone/bottom/image_raw",
		"/ardrone/bottom/image_raw/compressed",
		"/ardrone/front/image_raw",
		"/kobuki_safety_controller/reset",
		"/kobuki_safety_controller/disable",
		"/kobuki_safety_controller/enable",
		"/mobile_base/commands/reset_odometry"
	]

	$http({
		
		method: 'GET',
		url: Data.ip + "capabilities"
		
	}).then(function successCallback(response) {
		
		console.log("HELLO");
		var capabilities = response.data;
		
		angular.forEach(capabilities, function (messageCapability) {
			
			if($scope.topicToFilter.indexOf(messageCapability.topic) != -1) {
				
				console.log("Skipping " + messageCapability.topic)
				
			}
			else {
				
				Data.capabilities.push(messageCapability);
				
			}
			
		});
				
		$state.go("capabilities-ui");	
	
	}, function errorCallback(response) {
		
		console.log("Problems while contacting the robot" + response.status);
	
	});

}

function ontorobCtrl($scope, $http, $state, $compile,$interval, Data){
	
	$scope.blockUnselectedBackgroundColor = "#C0C0C0";
	$scope.blockSelectedBackgroundColor = "#ffb459";
	$scope.enabledBackgroundColor = "#FFFFFF"
	$scope.disabledBackgroundColor  = "#C0C0C0";
	
	$scope.program = {};
	$scope.program.sequence = [];
	$scope.program.selected = false;
	$scope.program.class = "program";
	$scope.program.backgroundColor = $scope.blockUnselectedBackgroundColor;
	
	$scope.blockCounter = 0;
	$scope.conditionCounter = 0;
	$scope.ip = Data.ip;
	$scope.streamIp = Data.streamIp;
	
	$scope.program.blockTypes = []
	initialiseBlockTypes($scope.program.blockTypes);
	
	// this hashmap is used to understand which class needs to be
	// disabled when an item in the interface is pressed
	
	
	$scope.disablingHash = {
		"condition":"parameter",
		"statement":"capability",
		"then":"block",
		"else":"block",
		"do":"block",
		"program":"block"
	}
	
	$scope.blockBackgroundsHash = {
		"statement":"#82ff9d",
		"if-then-else":"#ff9f82",
		"program":"#C0C0C0",
		"while-do":"#c1a0ff",
		"repeat":"#f7a5c7",
		"no-op":"#C4C4C4",
		"then":"#ff5e5e",
		"else":"#ff5e5e",
		"if":"#ff5e5e",
		"while":"#bc70e0",
		"condition":"#345ead",
		"do":"#e698f9"
	}
	
	// TODO the type of visualiser might be in the KB as well
	$scope.visualiserHash = {
		"Map_representation":{
			"display":"map_display.html",
			"refresh":180000
		},
		"Vision":{
			"display":"camera_display.html",
			"refresh":2000
		},
		"Robot_position":{
			"display":"basic_display.html",
			"refresh":3000
		},
		"Robot_speed":{
			"display":"basic_display.html",
			"refresh":3000
		},
		"Depth_Sensing":{
			"display":"basic_display.html",
			"refresh":3000
		},
		"Body_part_Movement":{
			"display":"basic_display.html",
			"refresh":3000
		},
		"ARTag_counting":{
			"display":"basic_display.html",
			"refresh":3000
		},
		"Imu_based_Robot_position":{
			"display":"basic_display.html",
			"refresh":3000
		},
		"Imu_based_Robot_speed":{
			"display":"basic_display.html",
			"refresh":3000
		},
		"Odometry_based_Robot_position":{
			"display":"basic_display.html",
			"refresh":3000
		},
		"Odometry_based_Robot_speed":{
			"display":"basic_display.html",
			"refresh":3000
		},
		"Trigger":{
			"display":"basic_display.html",
			"refresh":3000
		}
		
	}
	
	// TODO hardcoded capability descriptions
	// this should be in the KB
	$scope.capabilityDescriptions = {
		"Navigation":"Can be used to give to the robot a point to reach in a three-dimensional space. It expects the position as a three coordinates point (position.x, position.y, position.z) and the orientation as a quaternion (orientation.x, orientation.y, orientation.z, orientation.w). It also requires to specify the reference frame (frame_id:map).",
		"Odometry_based_Robot_speed":"It gives the current velocity of the robot. Provided as three linear velocities (linear.x, linear.y, linear.z) and three angular velocities (angular.x, angular.y, angular.z) with respect to the three main axes.",
		"Odometry_based_Robot_position":"It gives information on where the robot is located in a three-dimensional space. It uses a three coordinates point (position.x, position.y, position.z) to specify the position and a quaternion (orientation.x, orientation.y, orientation.z, orientation.w) for the orientation. It is derived from the wheel movement of the robot.",
		"Imu_based_Robot_position":"It gives information on the robot orientation in a three-dimensional space, specified as a quaternion (orientation.x, orientation.y, orientation.z, orientation.w). It is estimated from the Inertial Measurement Unit (IMU) measurements.",
		"Imu_based_Robot_speed":"It gives the current angular velocity (angular.x, angular.y, angular.z) and linear acceleration (linear.x, linear.y, linear.z) of the robot. Both are provided with respect to the three main axes.",
		"ARTag_counting":"It gives the current list of all the detected AR Tags (tags) and the total number of detetected tags (total).",
		"Map_representation":"A representation of the current map used by the robot. It is defined as a discrete grid where each cell can be empty (white), occupied (black) or unknown (grey).",
		"Depth_Sensing":"It provides the current mesaurement of the laser scanner, given as an array of distances detected by each ray (ranges).",
		"Directional_Movement":"Can be used to set a specific velocity to the robot. Provided as three linear velocities (linear.x, linear.y, linear.z) and three angular velocities (angular.x, angular.y, angular.z) with respect to the three main axes, x, y, z.",
		"Vision":"Video stream of the robot camera."
	}
	
	$scope.visualisersSettings = {}
	
	$scope.getVisualiser = function (capability) {
		$scope.visualiserHash[capability]["display"];
	}
	
	// this dictionary is used to sort capabilities to be read
	// at different time interval (e.g. a map will be read every 1 or 2 minutes)
	// while the robot position will be read every 5 seconds
	// entries are 2secs, 3secs, 5secs, 3 mins.
	// TODO I might remove this mechanism. Not sure I'll have
	// to update the map, for example (180000)
	$scope.readTimingHash = {
		2000:[],
		3000:[],
		5000:[],
		180000:[]
	}
	
	// this function is annoying. Not sure it will make the things betters
	$scope.requireReadings = function(capabilities) {
		
		toRequest = [];

		angular.forEach(capabilities, function (capabilityToRequest) {
			//console.log(capabilityToRequest.topic);
			// TODO filtering topic/capability, as their messages contain objects which need to be parsed
			if(capabilityToRequest.topic == "/move_base/global_costmap/obstacle_layer/clearing_endpoints" || capabilityToRequest.topic == "/move_base/local_costmap/obstacle_layer/clearing_endpoints") {
				//console.log("");
			}
			else {
				toRequest.push({"topic":capabilityToRequest.topic,"capability":capabilityToRequest.capability.type});
			}
			
		});

		$http({
			
			method: 'GET',
			url: $scope.ip + "read",
			params: {"question": angular.toJson(toRequest,false)}
			
		}).then(function successCallback(response) {
			
			// console.log(response.data);
			
			angular.forEach(capabilities, function(requestedCapability) {
				
				var key = requestedCapability.capability.type + "/" + requestedCapability.topic;				
				if(response.data.hasOwnProperty(key)) {
					
					var readParams = response.data[key];
					var curCapability = requestedCapability.capability;
					var curTopic = requestedCapability.topic;
					var curCapType = $scope.getNameFromURI(curCapability.type);

					if($scope.visualiserHash[curCapType]["display"] == "basic_display.html") {
				
						//console.log("Response for basic "+ response.status);					
						angular.forEach(curCapability.params, function(param) {
				
							paramName = $scope.getNameFromURI(param.p);
							param.value = readParams[paramName];
				
						});
		
					}
					// TODO this is a littble bit tailored on convetional maps
					else if($scope.visualiserHash[curCapType]["display"] == "map_display.html") {
						
						console.log("Response for map "+ response.status);
						
						$scope.visualisersSettings[key] = {
								"info.origin.position.x":readParams["info.origin.position.x"],
								"info.origin.position.y":readParams["info.origin.position.y"],
								"info.origin.position.z":readParams["info.origin.position.z"],
								"info.origin.orientation.w":readParams["info.origin.orientation.w"],
								"info.origin.orientation.x":readParams["info.origin.orientation.x"],
								"info.origin.orientation.y":readParams["info.origin.orientation.y"],
								"info.origin.orientation.z":readParams["info.origin.orientation.z"],
								"info.resolution":readParams["info.resolution"],
								"info.height":readParams["info.height"],
								"info.width":readParams["info.width"],
								"mouse.x":0,
								"mouse.y":0
							}
				
						var c = document.getElementById("map"+$scope.slashesToHyphens(curTopic));
						var ctx = c.getContext("2d");
						ctx.canvas.width  = readParams["info.width"];
						ctx.canvas.height = readParams["info.height"];
						//ctx.scale(2,2);
						var img = $scope.buildBmp(readParams['data'], readParams["info.width"], readParams["info.height"], ctx);
						ctx.putImageData(img, 0, 0);
						
						// TODO this is to make things sharper, to load the maps before starting streaming
						// can be done better
						angular.forEach($scope.readTimingHash[2000], function (cap) {
								
								console.log(cap.capability.type);
								cap.capability.display = true;
							
						});
				
					}
					else if($scope.visualiserHash[curCapType]["display"] == "camera_display.html") {
				
						console.log("Requesting camera data");
				
					}
					
				}
			});
			
	
		}, function errorCallback(response) {
			console.log("Error", toRequest);
			console.log(response);	
			console.log("Problems while contacting the KB server " + response.status);
			
		});
	}

	// this variable is used to keep track of the selected block in the program
	// I may use a function that retrieves it, as every block has a "selected" field
	// but I'll do a try with this one
	$scope.selectedBlock = $scope.program;
	
	console.log("Initial object");
	$scope.capabilities = Data.capabilities;
	console.log($scope.capabilities);
	
	$scope.modal =  {"text":"","header":""};
	
	$scope.getNameFromURI = function (uri) {
		
		return uri.substring(uri.lastIndexOf("/")+1,uri.length);
		
	}
	
	var counter = 0;
	
	angular.forEach($scope.capabilities,function(messageCapability) {
		
		capabilities = messageCapability["capabs"];
		
		angular.forEach(capabilities,function(capability) {
		
			capability.class = "capability";
			capability.topic = messageCapability.topic;
			capability.msg = messageCapability.msg;
			disable(capability);
	
			capability.hasReadParameter = false;
			
			// TODO part of the hardcoding of the descriptions
			capability.description = $scope.capabilityDescriptions[$scope.getNameFromURI(capability.type)];
			capability.showParameters = false;
			
			capability["params"].sort(function(a, b) {

				return a["p"].localeCompare(b["p"]);
		
			});

			angular.forEach(capability["params"],function(parameter) {
			
				parameter.class = "parameter";
				parameter.value = null;
				disable(parameter);
			
			});
		
			if(capability.mode == "read") {
			
				capName = $scope.getNameFromURI(capability.type);
			
				if($scope.visualiserHash[capName] != null) {
				
			
					refreshTime = $scope.visualiserHash[capName]["refresh"];
					$scope.readTimingHash[refreshTime].push({"topic":messageCapability.topic,"capability":capability});
			
				}
		
			}
		
		});
		
	});
	
	
	$scope.requireReadings($scope.readTimingHash[180000]);
	$interval($scope.requireReadings, 3000, 0, true, $scope.readTimingHash[3000]);

	// this function runs the program built so far
	// it "parses" the sequence of instantiated blocks and 
	// translates them into the needed json
	// TODO: check the updating of capability in the program panel
	// as they are all the same object
	// TODO I need to add an id to everything for the hashing
	$scope.run = function () {
		
		console.log("Running the current program");	
		//console.log($scope.program)
		
		//parse
		programObject = {"instructions":[]}

		// TODO: I need to run some consistency checking before
		angular.forEach($scope.program.sequence,function (block) {
			
			parsedInstruction = parseBlock(block);
			
			if(parsedInstruction != null) {
			
				programObject.instructions.push(parsedInstruction);
			
			}
		})
		
		console.log(programObject);
		
		//send
		$http({
			
			method: 'POST',
			url: $scope.ip + "execute",
			data: {"program": programObject}
			
		}).then(function successCallback(response) {
			
			console.log("Response "+ response.status);	
			
		}, function errorCallback(response) {
			
			console.log("Problems while contacting the KB server " + response.status);
			
		});
		
		$scope.conditionCounter = 0;

	}

	function parseBlock(block) {
		
		if(block.class == "statement") {
			
			return parseStatement(block);
		
		}
		else if(block.class == "if-then-else") {
		
			return parseIfThenElse(block);
		
		}
		else if(block.class == "while-do") {
			
			return parseWhileDo(block);
			
		}
		else if(block.class == "repeat") {
			
			return parseRepeat(block);
			
		}
		else if(block.class = "no-op") {
			
			// TODO this is horrible
			ret = {}
			ret["type"] = "noop";
			return ret;
			
		}
	}

	function parseStatement(statement) {
		
		var ret = {};
		ret["pkg"] = null;
		ret["name"] = null;
		ret["topic"] = statement.capability.topic;
		ret["fields"] = [];
		ret["capability"] = statement.capability.type;
		ret["type"] = "capability";

		angular.forEach(statement.capability.params, function (parameter) {
			
			if(parameter.mode == "write") {
				
				paramName = $scope.getNameFromURI(parameter.p);
				ret.fields.push(paramName);
				ret[paramName] = parameter.value;
			
			}
		
		});
		
		return ret;
		
	}
	
	function parseIfThenElse(ifThenElse) {
		
		var ret = {};
		ret.type = "if";
		ret.conditions = [];
		ret.then = [];
		ret.else = [];
		
		// parsing conditions
		angular.forEach(ifThenElse.conditions, function (condition) {
				
			parsedCondition = parseCondition(condition);
			
			if('length' in parsedCondition) {
				
				for(var i = 0; i < parsedCondition.length; i++) {
					
					ret.conditions.push(parsedCondition[i]);
					
				}
				
			}
			else {
				
				ret.conditions.push(parsedCondition);
				
			}
				
		});
		
		angular.forEach(ifThenElse.then.sequence, function (thenBlock) {
		
			parsedInstruction = parseBlock(thenBlock);
			
			if(parsedInstruction!= null) {
			
				ret.then.push(parsedInstruction);

			}
		
		});
		
		angular.forEach(ifThenElse.else.sequence, function (elseBlock) {
		
			parsedInstruction = parseBlock(elseBlock);
			
			if(parsedInstruction!= null) {
				
				ret.else.push(parsedInstruction);
				
			}
			
		});		
		
		return ret;
	}
	
	function parseWhileDo(whileDo) {
		
		var ret = {};
		ret.type = "while";
		ret.conditions = [];
		ret.do = [];

		// parsing conditions
		angular.forEach(whileDo.conditions, function (condition) {
				
			parsedCondition = parseCondition(condition);
			
			if('length' in parsedCondition) {
				
				for(var i = 0; i < parsedCondition.length; i++) {
					
					ret.conditions.push(parsedCondition[i]);
					
				}
				
			}
			else {
				
				ret.conditions.push(parsedCondition);
				
			}
				
		});
		
		angular.forEach(whileDo.do.sequence, function (doBlock) {
		
			parsedInstruction = parseBlock(doBlock);
			
			if(parsedInstruction != null) {
			
				ret.do.push(parsedInstruction);
			
			}
		
		});
		
		return ret;
	}
	
	function parseCondition(condition) {
		
		if(condition.logicOperator) {

			$scope.conditionCounter++;

			logicOp = {
				"id":$scope.conditionCounter,
				"type":"logicOperator",
				"value":condition.logicOperator
				
			};

			$scope.conditionCounter++;

			cond = {};
			cond["id"] = $scope.conditionCounter;			
			cond["type"] = "condition";
			cond["topic"] = condition.parameter.capability.topic
			cond["pkg"] = null;
			cond["name"] = null;
			cond["field"] = $scope.getNameFromURI(condition.parameter.parameter.p);
			cond["operator"] = condition.comparator;
			cond["val"] = condition.value;
			cond["not"] = condition.not;
		
			return [logicOp,cond];
		
		}
		else {

			$scope.conditionCounter++;

			cond = {};
			cond["id"] = $scope.conditionCounter;
			cond["type"] = "condition";
			cond["topic"] = condition.parameter.capability.topic
			cond["pkg"] = null;
			cond["name"] = null;
			cond["field"] = $scope.getNameFromURI(condition.parameter.parameter.p);
			cond["operator"] = condition.comparator;
			cond["val"] = condition.value;
			cond["not"] = condition.not;
			
			return cond;
		
		}
	}
	
	function parseRepeat(repeat) {
		
		var ret = {};
		ret.type = "repeat";
		ret.times = parseFloat(repeat.times);
		ret.do = [];
		
		angular.forEach(repeat.do.sequence, function (doBlock) {
		
			parsedInstruction = parseBlock(doBlock);
			
			if(parsedInstruction != null) {
			
				ret.do.push(parsedInstruction);
			
			}
		
		});
		
		return ret;
	}
	
	//
	$scope.addToProgram = function (block) {
		
		curBlock = {}
		curBlock.id = ++$scope.blockCounter;
		curBlock.class = block;
		curBlock.selected = false;
		curBlock.backgroundColor = $scope.blockBackgroundsHash[block];
		
		if(block == "statement") {
			
			console.log("Adding " + block + " statement");
			block.capability = null;
			
		}
		else if(block == "if-then-else") {
			
			console.log("Adding " + block + " statement");
			initialiseIfThenElse(curBlock);
			
		}
		else if(block == "while-do") {
			
			console.log("Adding " + block + " statement");
			initialiseWhileDo(curBlock);
			
		}
		else if(block == "repeat") {
			
			console.log("Adding " + block + " statement");
			initialiseRepeat(curBlock);
			
		}
		else if(block == "no-op") {
			
			console.log("Adding " + block + " statement");
			
		}
		
		$scope.selectedBlock.sequence.push(curBlock);
		
	}
	
	
	function initialiseIfThenElse(block) {
		
		block.then = {};
		block.then.class = "then";
		block.then.sequence = [];
		block.then.selected = false;
		//block.then.backgroundColor = $scope.blockUnselectedBackgroundColor; 
		block.then.backgroundColor = $scope.blockBackgroundsHash[block.then.class];

		block.else = {};
		block.else.class = "else";
		block.else.sequence = [];
		block.else.selected = false;
		//block.else.backgroundColor = $scope.blockUnselectedBackgroundColor;
		block.else.backgroundColor = $scope.blockBackgroundsHash[block.else.class];
		
		block.conditions = [];
		
		firstCondition = {}
		firstCondition.class = "condition";
		firstCondition.not = false;
		firstCondition.comparator = "==";
		firstCondition.logicOperator = false;
		firstCondition.value = null;
		firstCondition.selected = false;
		//firstCondition.backgroundColor = $scope.blockUnselectedBackgroundColor;
		// TODO condition background
		firstCondition.backgroundColor = $scope.blockBackgroundsHash[firstCondition.class];
		
		firstCondition.parameter = null;
		
		block.conditions.push(firstCondition);
	}

	function initialiseWhileDo(block) {
		
		block.do = {};
		block.do.class = "do";
		block.do.sequence = [];
		block.do.selected = false;
		//block.do.backgroundColor = $scope.blockUnselectedBackgroundColor;
		block.do.backgroundColor = $scope.blockBackgroundsHash[block.do.class];
		
		block.conditions = [];
		
		firstCondition = {}
		firstCondition.class = "condition";
		firstCondition.not = false;
		firstCondition.comparator = null;
		firstCondition.logicOperator = false;
		firstCondition.value = null;
		firstCondition.selected = false;
		//firstCondition.backgroundColor = $scope.blockUnselectedBackgroundColor;
		// TODO condition background
		firstCondition.backgroundColor = $scope.blockBackgroundsHash["condition"];
		
		firstCondition.parameter = null;
		
		block.conditions.push(firstCondition);
		
	}
	
	function initialiseRepeat(block) {
		
		block.times = 0;
		
		block.do = {};
		block.do.class = "do";
		block.do.sequence = [];
		block.do.selected = false;
		//block.do.backgroundColor = $scope.blockUnselectedBackgroundColor;
		block.do.backgroundColor = $scope.blockBackgroundsHash[block.do.class];
		
	}
	
	$scope.select = function (clickedBlock,$event) {
		
		$event.stopPropagation();
		//console.log(clickedBlock);
		$scope.selectedBlock = clickedBlock;
				
		clickedBlock.backgroundColor = $scope.blockSelectedBackgroundColor;
		clickedBlock.selected = true;
		
		// control the selection of the area
		if(clickedBlock.class != "program") {
			
			//deactivateUnselected($scope.program);
			$scope.program.backgroundColor = $scope.blockBackgroundsHash["program"];
			$scope.program.selected = false;
			
		}
		
		angular.forEach($scope.program.sequence, function (block) {
			
			deactivateUnselected(block, clickedBlock);
			
		});
		
		console.log(clickedBlock.class);
		disableAllBut($scope.disablingHash[clickedBlock.class]);
	}
	

	function deactivateUnselected(currentBlock, selectedBlock) {
		if(currentBlock == selectedBlock) {
			
			return
			
		}
		else if(currentBlock.class == "statement") {
			
			//currentBlock.backgroundColor = $scope.blockUnselectedBackgroundColor;
			currentBlock.backgroundColor = $scope.blockBackgroundsHash[currentBlock.class];
			currentBlock.selected = false;
			
		}
		else if(currentBlock.class == "then" || currentBlock.class == "else" || currentBlock.class == "do" || currentBlock.class == "program") {
			
			//currentBlock.backgroundColor = $scope.blockUnselectedBackgroundColor;
			currentBlock.backgroundColor = $scope.blockBackgroundsHash[currentBlock.class];
			currentBlock.selected = false;
			
		}
		else if(currentBlock.class == "condition"){
			
			//currentBlock.backgroundColor = $scope.blockUnselectedBackgroundColor;
			currentBlock.backgroundColor = $scope.blockBackgroundsHash[currentBlock.class];
			currentBlock.selected = false;
			
		}
		else if(currentBlock.class == "if-then-else") {
			
			deactivateUnselected(currentBlock.then, selectedBlock);
			
			angular.forEach(currentBlock.then.sequence, function (block){
				
				deactivateUnselected(block,selectedBlock);
				
			});
			
			deactivateUnselected(currentBlock.else, selectedBlock);
			
			angular.forEach(currentBlock.else.sequence, function (block) {
				
				deactivateUnselected(block,selectedBlock);
				
			});
			
			angular.forEach(currentBlock.conditions, function (block) {
				
				deactivateUnselected(block,selectedBlock);
				
			});
			
		}
		else if(currentBlock.class == "while-do") {
			
			deactivateUnselected(currentBlock.do, selectedBlock);
			
			angular.forEach(currentBlock.do.sequence, function (block){
				
				deactivateUnselected(block,selectedBlock);
				
			});
			
			angular.forEach(currentBlock.conditions, function (block) {
				
				deactivateUnselected(block,selectedBlock);
				
			});
			
		}
		else if(currentBlock.class == "repeat") {
			
			deactivateUnselected(currentBlock.do, selectedBlock);
			
			angular.forEach(currentBlock.do.sequence, function (block){
				
				deactivateUnselected(block,selectedBlock);
				
			});
			
		}

	}

	$scope.addCondition = function (block) {
				
		condition = {}
		condition.class = "condition";
		condition.not = false;
		condition.comparator = "==";
		
		if(block.conditions.length > 0) {
			
			condition.logicOperator = "and";
		
		}
		else {
			
			condition.logicOperator = false;
				
		}
		
		condition.value = null;
		condition.selected = false;
		//condition.backgroundColor = $scope.blockUnselectedBackgroundColor;
		condition.backgroundColor = $scope.blockBackgroundsHash[condition.class];
		
		condition.parameter = null;
		
		block.conditions.push(condition);
		
	}

	$scope.trigger = function (capability,topic,message,capabNameIndex){
		
		//capName = getNameFromURI($capabNameIndex);
		console.log("Sending action " + capability + " to the server");
		
		capabDiv = angular.element(document.getElementById("cap_"+capabNameIndex));
				
		//console.log(capabDiv.children());
		values = {};
		angular.forEach(capabDiv.children(),function(input) {
			
			inputElement = angular.element(input);
			
			if(inputElement[0].localName == "input") {
				
				//console.log(inputElement[0].value);
				values[inputElement[0].name] = inputElement[0].value;
				
			}
		});
		
		toSend = {};
		toSend["type"] = capability;
		toSend["topic"] = topic;
		toSend["message"] = message;
		toSend["parameters"] = values;
		
		console.log(toSend);
		
		$http({
			
			method: 'POST',
			url: $scope.ip + "trigger",
			data: {"capability": toSend}
			
		}).then(function successCallback(response) {
			
			console.log("Response "+ response.status);	
				
		}, function errorCallback(response) {
			
			console.log("Problems while contacting the KB server " + response.status);
			
		});
	}
	
	function disableAllBut (classToEnable) {
		
		angular.forEach($scope.capabilities,function(messageCapability) {
			
			capabilities = messageCapability["capabs"];
		
			angular.forEach(capabilities,function(capability) {
				
				if(classToEnable == "capability") {
					
					// TODO for now, it makes sense only to select
					// capability where you can write
					if(capability.mode == "write") {
						
						enable(capability);
					
					}
					
				}
				else {
					
					disable(capability);
				}
				
				angular.forEach(capability["params"],function(parameter) {
					
					if(classToEnable == "parameter") {

						enable(parameter);

					}
					else {
						
						disable(parameter);
						
					}
				});
			
			});
			
		});
		
		// checking also the blocks
		angular.forEach($scope.program.blockTypes, function(blockType) {
		
			if(classToEnable == "block") {
				
				enable(blockType);
				
			}
			else {
				
				disable(blockType);
				
			}
			
		});
	}
	
	function enable (item) {
		item.disabled = false;
		item.backgroundColor = $scope.enabledBackgroundColor;
	}
	
	function disable (item) {
		item.disabled = true;
		item.backgroundColor = $scope.disabledBackgroundColor;
	}
	
	function replaceSlashes(input) {
		
		return input.replace("/\\//g","-");
		
	}
	
	function initialiseBlockTypes(blockArray) {
		
		statement = {}
		statement.class = "statement";
		statement.name = "capability"
		statement.disabled = false;
		
		ifThenElse = {}
		ifThenElse.class = "if-then-else";
		ifThenElse.name = "if-then-else";
		ifThenElse.disabled = false;
		
		whileDo = {}
		whileDo.class = "while-do";
		whileDo.name = "while-do";
		whileDo.disabled = false;
		
		repeat = {}
		repeat.class = "repeat";
		repeat.name = "repeat";
		repeat.disabled = false;

		noOp = {}
		noOp.class = "no-op";
		noOp.name = "no-action";
		noOp.disabled = false;
		
		blockArray.push(statement);
		blockArray.push(ifThenElse);
		blockArray.push(whileDo);
		blockArray.push(repeat);
		blockArray.push(noOp);
	}
	
	$scope.addCapability = function (capability) {
		
		//console.log(capability);

		var copiedCapability = {};
		angular.copy(capability, copiedCapability);
		//console.log("copied");
		//console.log(copiedCapability);
		copiedCapability.$$hashKey = capability.$$hashKey;
		$scope.selectedBlock.capability = copiedCapability;
		
	}
	
	$scope.addParameter = function (parameter,capability) {
		
		var copiedParameter = {}
		var copiedCapability = {};
		angular.copy(capability, copiedCapability);
		angular.copy(parameter, copiedParameter);

		comparisonParameter = {}
		comparisonParameter.parameter = copiedParameter;
		comparisonParameter.capability = copiedCapability;
		$scope.selectedBlock.parameter = comparisonParameter;
		
	}
	
	$scope.stopPropagation = function ($event) {
		
		$event.stopPropagation();
		
	}
	
	$scope.removeInstruction = function (instruction,$event) {
		$event.stopPropagation();

		removeRecursively($scope.program.sequence,instruction);
		
	}
	
	function removeRecursively (instructionArray, instructionToRemove) {
		
		angular.forEach(instructionArray,function(curInstruction) {
			//console.log(instructionToRemove);
			//console.log(instructionToRemove.$$hashKey);
			//console.log(curInstruction);
			//console.log(curInstruction.$$hashKey);
			
			if(instructionToRemove.$$hashKey == curInstruction.$$hashKey) {
				
				console.log("YEAH, they're the same");
				var index = instructionArray.indexOf(instructionToRemove);
				removeFromInstructionArray(instructionArray,index);
				return true;
				
			}
			else {
				
				if(curInstruction.class == "statement") { 

					if(curInstruction.capability != null) {

						if(instructionToRemove.$$hashKey == curInstruction.capability.$$hashKey) {

							curInstruction.capability = null;
							
						}
						
					}
					
				}
				else if(curInstruction.class == "if-then-else") {

					if(removeRecursively(curInstruction.conditions,instructionToRemove)) {
						console.log("YEAH, if-then-else conditions");
						return true;
						
					}

					if(curInstruction.then.sequence.length > 0 && removeRecursively(curInstruction.then.sequence,instructionToRemove)) {
						console.log("YEAH, if-then-else then");
						return true;
						
					}

					if(curInstruction.else.sequence.length > 0 && removeRecursively(curInstruction.else.sequence,instructionToRemove)) {
						
						return true;
						
					}
						
				}
				else if(curInstruction.class == "while-do") {
					
					if(removeRecursively(curInstruction.conditions,instructionToRemove)) {
						
						return true;
						
					}
					if(curInstruction.do.sequence.length > 0 && removeRecursively(curInstruction.do.sequence,instructionToRemove)) {
						
						return true;
						
					}
				
				}
				else if(curInstruction.class == "repeat") {
					
					if(curInstruction.do.sequence.length > 0 && removeRecursively(curInstruction.do.sequence,instructionToRemove)) {
						
						return true;
						
					}
					
				}
				
			}
			
			return false;
				
		});
		
	}
	
	function removeFromInstructionArray(instructionArray,index) {
		
		console.log(instructionArray[index]);
		instructionArray.splice(index,1);
		
	}
	
	$scope.showDescription = function(capability) {
		
		$scope.modal.header = $scope.getNameFromURI(capability.type);
		$scope.modal.text = capability.description;
		
	}
	
	$scope.buildBmp = function (matrix, width, height, context) {
		
		var bpmMatrix = [];
		var gray = [168,168,168];
		var black = [0,0,0];
		var white = [255,255,255]
		var imgData = context.createImageData(width,height);
		
		var pixelIndex = 0;
		for(var p = 0; p < matrix.length; ++p) {
			
			var color = null;
				
			if(matrix[p] == -1) {
				color = gray;
			}
			else {
				evaluatedColor = -(matrix[p]*2.55 - 255);
				color = [evaluatedColor,evaluatedColor,evaluatedColor];
			}
			
			// red
			imgData.data[pixelIndex] = color[0];
			pixelIndex++;
			//green
			imgData.data[pixelIndex] = color[1];
			pixelIndex++;
			//blue
			imgData.data[pixelIndex] = color[2];
			pixelIndex++;
			//alpha
			imgData.data[pixelIndex] = 255;
			pixelIndex++;
		}
		
		return imgData;
	}

	$scope.zoomOut = function(){
		console.log("Halli");
	}
	$scope.zoomIn = function(){
		console.log("Hallo");
	}
	
	
	$scope.getMousePos = function ($event, cap, topic) {

		key = cap+"/"+topic;
		
		curSettings = $scope.visualisersSettings[key];
		
		if(curSettings != null) {
			
	  	  	let rect = $event.target.getBoundingClientRect(),
	        x = $event.clientX - rect.left,
	        y = $event.clientY - rect.top;
    		// console.log(curSettings["info.origin.position.x"]);
			// console.log(curSettings["info.origin.position.y"]);
			finalx = (x - curSettings["info.width"]/2+curSettings["info.origin.position.x"])*curSettings["info.resolution"];
			finaly = (y - curSettings["info.height"]/2+curSettings["info.origin.position.y"])*curSettings["info.resolution"];
		
			curSettings["mouse.x"] = finalx;
			curSettings["mouse.y"] = finaly;
		}
		
	}

	$scope.slashesToHyphens = function (string) {
		return string.replace(/\//g , "-");
	}

	$scope.showParameters = function (capability) {
		
		capability.showParameters = true;
		
	}
	
	$scope.hideParameters = function (capability) {
		
		capability.showParameters = false;
		
	}

}



