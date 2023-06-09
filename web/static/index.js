var car_vm;

let ros = new ROSLIB.Ros({
  url : 'ws://'+window.location.hostname+':9090'
  });


ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});


angular.module("car",[]).controller("CarController", function($scope, $http, $timeout, $log, $q, $filter) {
  var vm = this;
  vm.Math = Math;
  vm.JSON = JSON;



  vm.type_of = function (val) {
    return typeof (val);
  };

  vm.is_changed = function (field) {
    if (field.original_value === null != field.value === null) {
      return true;
    }
    if (field.original_value === null ) {
      return false;
    }

    return field.original_value.toString() !== field.value.toString();

  };

  vm.to_field_array = function (x) {
    var rv = [];
    for (var k in x) {
      rv.push({
        key: k,
        value: x[k],
        original_value: x[k],
        data_type: typeof (x[k])
      });
    }
    return rv;
  };

  vm.set_changes_from_field_array = function (o, field_array) {
    for (var i in field_array) {
      var field = field_array[i];
      if (vm.is_changed(field)) {
        if (field.data_type == 'number') {
          o[field.key] = Number(field.value);
        } else if (field.data_type == 'boolean') {
          o[field.key] = Boolean(field.value);
        } else {
          $log.error("unknown data type" + field.data_type);
        }
      }
    }
  };

  var topic = new ROSLIB.Topic({
    ros : ros,
    name : '/cmd_vel2',
    messageType : 'geometry_msgs/Twist'
  });

  vm.poweroff = function () {
    var twist = new ROSLIB.Message({
      linear: {
        x: this.Math.random()*0.1
      },
      angular: {
        z: Math.random()*0.1
      }
    });
    topic.publish(twist);
    return;
      vm.go_error = "";
    $http.put('/pi/poweroff', "1").success(function () {
      $log.info('poweroff success');
    }).error(function (response, code) {
      vm.go_error = "  (" + code + ")" + response.message;
    });
    $log.info("poweroff clicked");
  };

  vm.add_track = function() {
    var track_name = $filter('date')(new Date(),'yyyy-MM-ddThh:mm:ss');
    var req = {
      url: '/tracks/'+encodeURIComponent(track_name),
      method: 'PUT',
      data: angular.toJson({"name":track_name}),
      headers: {
        'Content-Type': 'application/json'
      }
    };
    $http(req).then(function() {
      $log.info('add track success');
      vm.refresh_track_names();
    }).catch(function(response, code){
      $log.warn('add track failed');
      $log.warn("  (" + code + ")" + response);
    });

  };


  vm.save_run_settings = function (include_route_path = true) {
    var deferred = $q.defer();
    vm.go_error = "";
    vm.set_changes_from_field_array(vm.run_settings, vm.run_settings_array);
    var req = {
      url: '/run_settings',
      method: 'PUT',
      data: angular.toJson(vm.run_settings),
      headers: {
        'Content-Type': 'application/json'
      }
    };

    var req2 = {
      url: vm.route_path_url(),
      method: 'PUT',
      data: angular.toJson(vm.route_path),
      headers: {
        'Content-Type': 'application/json'
      }
    };

    $http(req).success(function () {
      $log.info('save run settings success');
      if(include_route_path) {
        $http(req2).success(function () {
          deferred.resolve();
          $log.info('save route path success');
        }).error(function (response, code) {
          $log.warn("failed to save route path");
          $log.warn("  (" + code + ")" + response);
          deferred.reject(response);
        });
      } else {
        deferred.resolve();
      }
    }).error(function (response, code) {
      vm.go_error = "  (" + code + ")" + response;
      deferred.reject(response);
    });

    $log.info("save_run_settings clicked");
    return deferred.promise;

  };


  vm.go = function () {
    vm.save_run_settings().then(function() {
      vm.go_error = "";
      $http.put('/command/go', "1").success(function () {
        $log.info('go success');
      }).error(function (response, code) {
        vm.go_error = "  (" + code + ")" + response.message;
      });
      $log.info("go clicked");
    });
  };

  vm.stop = function () {
    $http.put('/command/stop', "1").success(function () {
      $log.info('stop success');
      $timeout(function(){vm.refresh_routes(true);},3000); // timeout gives server time to save route
    }).error(function (response, code) {
      vm.stop_error = "  (" + code + ")" + response.message;
    });
    $log.info("stop clicked");
  };

  vm.record = function () {
    vm.save_run_settings(false).then(function() { // save run settings first so recording knows where to go
      $http.put('/command/record', "1").success(function () {
        $log.info('record success');
      }).error(function (response, code) {
        vm.record_error = "  (" + code + ")" + response.message;
      });
    });
  $log.info("record clicked");
  };


   var reset_odom_service = new ROSLIB.Service({
     ros : ros,
     name : '/car/reset',
     serviceType : 'std_srvs/Empty'
   });

  vm.reset_odometer = function () {
    var request = new ROSLIB.ServiceRequest({
    });
  
    reset_odom_service.callService(request, function(result) {
      console.log("reset odom completed");
    });
  };

  var start_motor_service = new ROSLIB.Service({
    ros : ros,
    name : '/car/start_scan',
    serviceType : 'std_srvs/Empty'
  });

  vm.start_motor = function () {
    var request = new ROSLIB.ServiceRequest({
    });
  
    start_motor_service.callService(request, function(result) {
      console.log("start_motor completed");
    });
  };

  var stop_motor_service = new ROSLIB.Service({
    ros : ros,
    name : '/car/stop_scan',
    serviceType : 'std_srvs/Empty'
  });


  vm.stop_motor = function () {
    var request = new ROSLIB.ServiceRequest({
    });
  
    stop_motor_service.callService(request, function(result) {
      console.log("stop_motor completed");
    });
  };

  var enable_ros_control_service = new ROSLIB.Service({
    ros : ros,
    name : '/car/enable_ros_control',
    serviceType : 'std_srvs/Empty'
  });

  var disable_ros_control_service = new ROSLIB.Service({
    ros : ros,
    name : '/car/disable_ros_control',
    serviceType : 'std_srvs/Empty'
  });

  vm.enable_ros_control = function () {
    var request = new ROSLIB.ServiceRequest({
    });
  
    enable_ros_control_service.callService(request, function(result) {
      console.log("enable_ros_control completed");
    });
  };

  vm.disable_ros_control = function () {
    var request = new ROSLIB.ServiceRequest({
    });
  
    disable_ros_control_service.callService(request, function(result) {
      console.log("disable_ros_control completed");
    });
  };



  vm.reset_zoom = function () {
    // viewbox is a square centered at center of route  
    var size = Math.max(vm.route_path_width + 0.8, vm.route_path_height);
    vm.viewbox_left = ((vm.route_path_max_x+vm.route_path_min_x)/2 - size/2);
    vm.viewbox_top = -((vm.route_path_max_y+vm.route_path_min_y)/2 + size/2);
    vm.viewbox_width = size;
    vm.viewbox_height = size;
  };


  vm.pan_delta = 0.1;
  vm.zoom_ratio = Math.sqrt(2);

  vm.zoom_in = function () {
    var center_x = vm.viewbox_left + vm.viewbox_width / 2.0;
    var center_y = vm.viewbox_top + vm.viewbox_height / 2.0;

    vm.viewbox_height /= vm.zoom_ratio;
    vm.viewbox_width /= vm.zoom_ratio;

    vm.viewbox_top = center_y - vm.viewbox_height / 2;
    vm.viewbox_left = center_x - vm.viewbox_width / 2;
  };

  vm.zoom_out = function () {
    var center_x = vm.viewbox_left + vm.viewbox_width / 2.0;
    var center_y = vm.viewbox_top + vm.viewbox_height / 2.0;

    vm.viewbox_height *= vm.zoom_ratio;
    vm.viewbox_width *= vm.zoom_ratio;

    vm.viewbox_top = center_y - vm.viewbox_height / 2;
    vm.viewbox_left = center_x - vm.viewbox_width / 2;
  };

  vm.pan_left = function () {
    vm.viewbox_left -= vm.viewbox_width * vm.pan_delta;
  };

  vm.pan_right = function () {
    vm.viewbox_left += vm.viewbox_width * vm.pan_delta;
  };

  vm.pan_up = function () {
    vm.viewbox_top -= vm.viewbox_height * vm.pan_delta;
  };

  vm.pan_down = function () {
    vm.viewbox_top += vm.viewbox_height * vm.pan_delta;
  };

  vm.refresh_routes = function(select_latest) {
    $log.info('select_latest' + select_latest);
    $http.get('/tracks/' + encodeURIComponent(vm.run_settings.track_name) + "/routes").
      success(function (result /*, status, headers, config*/) {
        vm.routes = result.routes;
        vm.route_names = [];
        var max_time = '';
        for(var i in vm.routes) {
          var route = vm.routes[i];
          vm.route_names.push(route.name);
          if(select_latest && route.time > max_time) {
            max_time = route.time;
            vm.run_settings.route_name = route.name;
          }
        }
      }).error(function (/*data, status, headers, config*/) {
        vm.routes = [''];
        vm.route_names = [''];
        // log error
      });
  };

  vm.refresh_runs = function(select_latest) {
    $log.info('select_latest' + select_latest);
    $http.get('/tracks/' + encodeURIComponent(vm.run_settings.track_name) 
         + "/routes/" + encodeURIComponent(vm.run_settings.route_name)
         + "/runs"
         ).
      success(function (result /*, status, headers, config*/) {
        vm.runs = result.runs
        vm.run_names = [];
        var max_time = '';
        for(var i in vm.runs) {
          var run = vm.runs[i];
          vm.run_names.push(run.name);
          if(select_latest && run.time > max_time) {
            max_time = run.time;
            vm.run_settings.run_name = run.name;
          }
        }
      }).error(function (/*data, status, headers, config*/) {
        vm.runs = [''];
        vm.runs = [''];
        // log error
      });
  };

  $scope.$watch("car.run_settings.track_name", function () {
    if (vm.run_settings && vm.run_settings.track_name !== null && vm.run_settings.track_name.length > 0) {
      $log.info("selected track:", vm.run_settings.track_name);
      vm.refresh_routes();
    }
  });

  vm.route_path_url = function () {
    return '/tracks/' + encodeURIComponent(vm.run_settings.track_name) + "/routes/" + encodeURIComponent(vm.run_settings.route_name) + "/path";
  };

  vm.run_path_url = function () {
    return '/tracks/' + encodeURIComponent(vm.run_settings.track_name) + "/routes/" + encodeURIComponent(vm.run_settings.route_name) + "/runs/" + encodeURIComponent(vm.run_settings.run_name) +"/path";
  };

  vm.has_road_sign = function(node) {
    if ( angular.isDefined(node.road_sign_label) && node.road_sign_label && node.road_sign_label.length > 0 ){
      return true;
    }
    if ( angular.isDefined(node.road_sign_command) && node.road_sign_command && node.road_sign_command.length > 0 ) {
      return true;
    }
    return false;
  };

  vm.populate_road_sign_nodes = function () {
    try {
      viewer.set_path(vm.route_path);
    } catch {} // in case not defined
    
    vm.road_sign_nodes = [];
    for (var i in vm.route_path) {
      var node = vm.route_path[i];
      if (vm.has_road_sign(node)) {
        vm.road_sign_nodes.push(node);
      }
    }
    
  };

  $scope.$watch("car.run_settings.route_name", function () {
    if (vm.run_settings && vm.run_settings.route_name !== null && vm.run_settings.route_name.length > 0) {
      vm.refresh_runs(true);
      $http.get(vm.route_path_url()).
        success(function (route_path /*, status, headers, config*/) {
          viewer.set_path(route_path);
          vm.route_path = route_path;
          /*
          var route_x = route_path.map(function (v) { return v.x; });
          var route_y = route_path.map(function (v) { return v.y; });

          vm.route_path_min_x = Math.min.apply(null, route_x);
          vm.route_path_min_y = Math.min.apply(null, route_y);
          vm.route_path_max_x = Math.max.apply(null, route_x);
          vm.route_path_max_y = Math.max.apply(null, route_y);
          vm.route_path_width = vm.route_path_max_x - vm.route_path_min_x;
          vm.route_path_height = vm.route_path_max_y - vm.route_path_min_y;
          vm.reset_zoom();
          
          vm.populate_road_sign_nodes();
          */

        }).error(function (/*data, status, headers, config*/) {
          vm.route_names = ['n/a'];
          $log.error('$http failed to get route path');
        });
    }
  });

  $scope.$watch("car.run_settings.run_name", function () {
    if (vm.run_settings && vm.run_settings.route_name !== null && vm.run_settings.route_name.length > 0) {
      $http.get(vm.run_path_url()).
        success(function (run_path /*, status, headers, config*/) {
          viewer.set_run_path(run_path);
          vm.run_path = run_path;
        }).error(function (/*data, status, headers, config*/) {
          vm.route_names = ['n/a'];
          $log.error('$http failed to get route path');
        });
    }
  });


  vm.refresh_track_names = function() {
    vm.track_names = [];
    vm.track_name = "";
    $http.get('/track_names').
      success(function ( result) {
        vm.track_names = result.track_names;
      }).
      error(function () {
        $log.error('$http failed to get track names');
      });
    vm.poll_ok = false;
    vm.display_car_state = false;
  };

  vm.refresh_track_names();


  // var poller = function () {
  //   $http({ method: 'GET', timeout: 5000, url: '/car/get_state' })
  //     .then(function (r) {
  //       let car_state = r.data;
  //       viewer.set_car_state(car_state);
  //       vm.car_state = car_state;
  //       var v_bat = vm.car_state.v_bat;
  //       var min_bat = 3.5 * 3;
  //       var max_bat = 4.2 * 3;
  //       vm.battery_percent = 100 * (v_bat - min_bat) / (max_bat - min_bat);
  //       vm.poll_ok = true;
  //       $timeout(poller, 100);
  //     },
  //     function () {
  //       vm.poll_ok = false;
  //       $timeout(poller, 1000);
  //     });
  // };
  // poller();


  var int_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/my_int2',
    messageType: 'std_msgs/msg/Int32',
    queue_length : 1,
    throttle_rate: 1000

    
    // messageType : 'std_msgs/String'
  });

  
  int_listener.subscribe(function(message) {
    console.log(message);
    // vm.battery_percent = message.percentage * 100;
  });


  
  var battery_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/car/battery',
    messageType: 'sensor_msgs/BatteryState',
    queue_length : 1,
    throttle_rate: 1000

    
    // messageType : 'std_msgs/String'
  });

  
  battery_listener.subscribe(function(message) {
    vm.battery = message;
    // vm.battery_percent = message.percentage * 100;
  });

  var tf_client = new ROSLIB.TFClient({
    ros : ros,
    fixedFrame : "odom",
    rate: 100,
    angularThres : 0.00001,
     transThres : 0.00001
  });

  vm.tf_count = 0;

  tf_client.subscribe('base_footprint', function(msg) {
    ++vm.tf_count;
    vm.base_link = msg;

    let quat = new THREE.Quaternion(
      msg.rotation.x,
      msg.rotation.y,
      msg.rotation.z,
      msg.rotation.w
    );
    let euler = new THREE.Euler();
    euler.setFromQuaternion(quat);
    vm.base_link.yaw = euler.z;

    viewer.set_car_state(vm.base_link.translation, vm.base_link.yaw);
    

  });


  vm.speedometers = [];
  var fr_subscriber = new ROSLIB.Topic({
    ros : ros,
    name : '/car/speedometers/fr',
    messageType: 'car_msgs/msg/Speedometer',
    queue_length : 1,
    throttle_rate: 100
  });
  fr_subscriber.subscribe(function(message) {
    vm.speedometers.fr = message;
    get_car_scope().$apply();
  });

  new ROSLIB.Topic({
    ros : ros,
    name : '/car/speedometers/fl',
    queue_length : 1,
    throttle_rate: 100,
    messageType : 'car_msgs/msg/Speedometer'
  }).subscribe(function(message) {
    vm.speedometers.fl = message;
    get_car_scope().$apply();
  });

  new ROSLIB.Topic({
    ros : ros,
    name : '/car/speedometers/motor',
    queue_length : 1,
    throttle_rate: 100,
    messageType : 'car_msgs/msg/Speedometer'
  }).subscribe(function(message) {
    vm.speedometers.motor = message;
    get_car_scope().$apply();
  });



  vm.road_sign_nodes = [];

  vm.rjust = function (s, width) {
    if (angular.isUndefined(s)) {
      return " ".repeat(width);
    }
    s = s.toString();
    return " ".repeat(width - s.length) + s;
  };

  vm.property_list = function (o) {
    var s = [];
    for (var property_name in o) {
      s.push(property_name);
    }
    return s.join(", ");
  };

  vm.delete_road_sign = function (road_sign_index) {
    var node = vm.road_sign_nodes[road_sign_index];
    node.road_sign_label = "";
    node.road_sign_command = "";
    node.arg1 = "";
    node.arg2 = "";
    node.arg3 = "";
    vm.populate_road_sign_nodes();
  };

  vm.node_clicked = function ($event, node) {
    vm.selected_node = node;
    if (!vm.has_road_sign(node)) {
      node.road_sign_label = vm.road_sign_nodes.length.toString();
      node.road_sign_command = "stop";
      vm.populate_road_sign_nodes();
    }
    return;
  };

  // var poller2 = function () {
  //   $http({ method: 'GET', timeout: 5000, url: '/pi/get_state' })
  //     .then(function (r) {
  //       vm.pi_state = r.data;
  //       $timeout(poller2, 1000);
  //     },
  //     function () {
  //       $timeout(poller2, 1000);
  //     });
  // };
  // poller2();

  // var poller3 = function () {
  //   $http({ method: 'GET', timeout: 5000, url: '/run_settings' })
  //     .then(function (r) {
  //       vm.run_settings = r.data;
  //       vm.run_settings_array = vm.to_field_array(vm.run_settings);
  //     },
  //     function () {
  //       $timeout(poller3, 1000);
  //     });
  // };
  // poller3();
  car_vm = vm; // expose this angular stuff to viewer
});
