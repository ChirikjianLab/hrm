/*********************************************************************
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
 *********************************************************************/

/* Author: Wim Meeussen
 *
 * Modified: Pouya Mohammadi
 *   This is based on ROS stack and code by Wim, which I unROSed!
 *   There are prbably some unused functions and there might be some errors in
 *   some cases. Use this at your own risk!
 *
 * Further modified: Sipu Ruan
 *   changed boost::shared_ptr into urdf::*ShredPtr
 */

#include "kdl_parser/model.h"

/* include the default parser for plain URDF files;
   other parsers are loaded via plugins (if available) */
#include <urdf_parser/urdf_parser.h>

#include <fstream>
#include <iostream>

namespace urdf {

bool Model::initXml(TiXmlDocument* xml) {
    if (xml == nullptr) {
        std::cout << "Could not parse the xml document" << std::endl;
        return false;
    }

    std::stringstream ss;
    ss << *xml;

    return Model::initString(ss.str());
}

// My take on that method. It is however, necessary to ckeck if COLLADA is
// needed in some cases...
bool Model::initString(const std::string& xml_string) {
    ModelInterfaceSharedPtr model;

    // necessary for COLLADA compatibility
    model = parseURDF(xml_string);

    // copy data from model into this object
    if (model) {
        this->links_ = model->links_;
        this->joints_ = model->joints_;
        this->materials_ = model->materials_;
        this->name_ = model->name_;
        this->root_link_ = model->root_link_;
        return true;
    }
    return false;
}

}  // namespace urdf
