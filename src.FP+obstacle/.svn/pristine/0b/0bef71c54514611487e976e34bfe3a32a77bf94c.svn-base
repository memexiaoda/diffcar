/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef WEBSOCKET_SERVER_H_
#define WEBSOCKET_SERVER_H_

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
// Custom logger
#include <websocketpp/logger/syslog.hpp>
#include <iostream>

#include <boost/lockfree/queue.hpp>
#include <boost/lockfree/policies.hpp>

#include <random>
#include <iostream>
#include <jsoncpp/json/json.h>

#include <std_msgs/String.h>
#include <string>
#include <vector>

//using websocketpp::lib::placeholders::_1;
//using websocketpp::lib::placeholders::_2;
//using websocketpp::lib::bind ;

#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#ifdef __cplusplus
extern "C" {
#endif


namespace move_base
{
	typedef websocketpp::server<websocketpp::config::asio> server;
	// pull out the type of messages sent by our config
	typedef server::message_ptr message_ptr;


	struct msg_info{
		message_ptr message;
		websocketpp::connection_hdl client_hdl;
	};


	static std::string _message;
	static websocketpp::connection_hdl client_id;
	static std::vector<websocketpp::connection_hdl> client_list;

    server echo_server;
	boost::lockfree::queue<msg_info*,boost::lockfree::fixed_sized<true>> que_msg(1000);

	class WebsocketServer
	{

	public:
		  WebsocketServer(){};
		  ~WebsocketServer(){
			  if(!client_list.empty()){
				  client_list.clear();
			  }
		  };

	public:
		  std::string local_message;
		  std::string get_local_message(){
			  local_message = _message;
			  return local_message;
		  }

		  static void on_http(server* s, websocketpp::connection_hdl hdl) {
	          server::connection_ptr con = s->get_con_from_hdl(hdl);
	          std::string res = con->get_request_body();
	          std::stringstream ss;
	          ss << "got HTTP request with " << res.size() << " bytes of body data.";
	          con->set_body(ss.str());
	          con->set_status(websocketpp::http::status_code::ok);
	          std::cout << "uri" << con->get_uri() << std::endl;
	      }

	      static bool validate(server *, websocketpp::connection_hdl) {
	          //sleep(6);
	          return true;
	      }

	      static void on_fail(server* s, websocketpp::connection_hdl hdl) {
		      server::connection_ptr con = s->get_con_from_hdl(hdl);
		      std::cout << "Fail handler: " << con->get_ec() << " " << con->get_ec().message()  << std::endl;
		  }

		  static void on_close(websocketpp::connection_hdl) {
		      std::cout << "Close handler" << std::endl;
		  }

		  // Define a callback to handle incoming messages
		  static void on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg) {

			  msg_info* p_msg_info = new msg_info;

			  p_msg_info->message = msg;
			  p_msg_info->client_hdl = hdl;
			  bool is_newclient = true;

			  int client_count = 0;

			  while(client_count < client_list.size()){

				  if(client_list[client_count].lock().get() == hdl.lock().get()){
					  is_newclient = false;
				  }
				  client_count++;
			  }

			  if (is_newclient){
				  client_list.push_back(hdl);
			  }

			  que_msg.push(p_msg_info);

			  std::cout << "client_hdl =" << p_msg_info->client_hdl.lock().get() << "  message:"<< (p_msg_info->message)->get_payload() << std::endl;

			  // check for a special command to instruct the server to stop listening so
			  // it can be cleanly exited.
			  if (msg->get_payload() == "stop-listening") {
				  s->stop_listening();
				  return;
			  }
		  }
	};
};

#ifdef __cplusplus
}
#endif


#endif
