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
.controller('ontorobCtrl', ['$scope', '$http','$state','$compile','Data', ontorobCtrl])
.controller('indexCtrl', ['$scope', '$http','$timeout','$window','$state','Data', indexCtrl])
.service("Data", dataService);

function dataService () {
	this.capabilities = {};
}

function indexCtrl($scope,$http,$timeout,$window,$state, Data) {
	$scope.errorUrl = "error.html";
	$scope.successUrl = "capabilities-ui.html";

	$http({
		method: 'GET',
		//url: "http://localhost:5000/capabilities",
		url: "http://137.108.122.193:5000/capabilities"
		//url: "http://10.229.169.122:5000/capabilities"
	}).then(function successCallback(response) {
		//$scope.capabilities = response.data;
		Data.capabilities = response.data; 
		console.log("I should redirect now");
		console.log(Data.capabilities);
		$state.go("capabilities-ui");	
	
	}, function errorCallback(response) {
		console.log("Problems while contacting the robot" + response.status);
	
	});
}

function ontorobCtrl($scope, $http, $state, $compile, Data){
	console.log("Initial object");
	$scope.capabilities = Data.capabilities;
	console.log($scope.capabilities);
	target = document.getElementById('capability_container');
	
	// for each message
	counter = 0
	angular.forEach($scope.capabilities,function(item) {
		console.log("Capabilities");
		console.log(item);
		curCapabs = item["capabs"];
		curTopic = item["topic"];
		curMsg = item["msg"];
		capabHasParameter = false;
		
		// for each capability
		angular.forEach(curCapabs,function(capab) {
			if(capab["params"].length > 0) {
				console.log("Capability");
				console.log(capab);
			}
			capabNameURI = capab["type"];
			
			capabName = getNameFromURI(capabNameURI);
			capabName_index = capabName+"_"+counter;
			newCapabDiv = angular.element("<div id='cap_"+capabName_index+"'></div>");
			angular.element(target).append(newCapabDiv);
			
			newCapabH3= angular.element("<h3>"+capabName+" ("+curTopic+" - "+ curMsg +")</h3>");
			angular.element(newCapabDiv).append(newCapabH3);
			
			//newCapabHiddenTopicInput = angular.element("<input id='topic_"+replaceSlashes(curTopic)+"' type='hidden' value='"+curTopic+"'/>");
			//newCapabHiddenMsgInput = angular.element("<input id='msg_"+replaceSlashes(curMsg)+"' type='hidden' value='"+curMsg+"'/>");
			
			console.log(capab["params"][0]["p"]);
			console.log(typeof capab["params"][0]["p"]);
			
			capab["params"].sort(function(a, b) {
				return a["p"].localeCompare(b["p"]);
			});
			
			// for each capability parameters
			angular.forEach(capab["params"],function(parameter) {
				console.log("Parameter");
				console.log(parameter);
				mode = parameter["mode"];
				
				targetCapab = document.getElementById('cap_'+capabName_index);
				capabHasParameter = true;
				if(mode === "write") {
					paramNameURI = parameter["p"];
					paramNameDotted = getNameFromURI(paramNameURI);
					paramName = paramNameDotted.substring(paramNameDotted.lastIndexOf(".")+1,paramNameDotted.length);
					newParamNameElement = angular.element("<span>"+paramNameDotted+" </span>");
					newParamInput = angular.element("<input id='param_"+paramName+"_"+capabName_index+"' type='text' name='"+paramNameDotted+"'/>");
					br = angular.element("<br/>");
					angular.element(targetCapab).append(newParamNameElement);
					angular.element(targetCapab).append(newParamInput);
					angular.element(targetCapab).append(br);
				}
			});
			
			if(capabHasParameter) {
				submitButton = angular.element("<button type='button' class='btn navbar-btn btn-custom pull-left' ng-click='trigger( \""+capabNameURI+"\",\""+curTopic+"\",\""+curMsg+"\",\""+capabName_index+"\")'>Send action</button>");
				angular.element(targetCapab).append(submitButton);
				$compile(submitButton)($scope);
			}
			hr = angular.element("<hr/>");
			angular.element(targetCapab).append(hr);
			counter++;
		});
	});
	
	$scope.trigger = function (capability,topic,message,capabNameIndex){
		console.log("boya! " + capabNameIndex);
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
			//url: "http://localhost:5000/trigger",
			url: "http://137.108.122.193:5000/trigger",
			//url: "http://10.229.169.122:5000/trigger",
			data: {"capability": toSend}
		}).then(function successCallback(response) {
			console.log("Response "+ response.status);		
		}, function errorCallback(response) {
			console.log("Problems while contacting the KB server " + response.status);
			
		});
	}
	
	function getNameFromURI(uri) {
		return uri.substring(uri.lastIndexOf("/")+1,uri.length);
	}
	
	function replaceSlashes(input) {
		return input.replace("/\\//g","-");
	}
}


