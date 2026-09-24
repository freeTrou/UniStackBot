#ifndef UNISTACKBOT_CONTROLLER__URDF_COMMAND_JOINTS_HPP_
#define UNISTACKBOT_CONTROLLER__URDF_COMMAND_JOINTS_HPP_

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <tinyxml2.h>

#include "unistackbot_interface/joint_capacity.hpp"

namespace unistackbot_controller
{

/*
 * 解析 URDF <ros2_control> 块: position 命令关节表 + min/max/max_velocity。
 * 单一事实源: 限位与 BackendKinematic / CM 控制器同参数同缺省。
 * (2026-09-23 从 joint_stream_controller 匿名命名空间提取共享 —— OTG 门同款消费,
 *  关节契约解析必须单源; 函数体逐行保持不变)
 * 上界 fail-fast (2026-09-23 审查补): 消费方 JS/门用 kMaxJoints 定长数组按序写入,
 * 17+ 命令关节的 URDF 会在配置期越界 —— 在唯一入口拒绝, 不留给运行期。
 */
inline bool parseJointsFromUrdf(
	const std::string & urdf, std::vector<std::string> & names,
	std::vector<double> & qmin, std::vector<double> & qmax,
	std::vector<double> & vmax, std::string & err)
{
	tinyxml2::XMLDocument doc;
	if (doc.Parse(urdf.c_str(), urdf.size()) != tinyxml2::XML_SUCCESS)
	{
		err = "URDF XML 解析失败";
		return false;
	}
	const tinyxml2::XMLElement * root = doc.RootElement();
	if (root == nullptr)
	{
		err = "URDF 无根元素";
		return false;
	}
	for (const tinyxml2::XMLElement * rc = root->FirstChildElement("ros2_control"); rc != nullptr;
		rc = rc->NextSiblingElement("ros2_control"))
	{
		for (const tinyxml2::XMLElement * j = rc->FirstChildElement("joint"); j != nullptr;
			j = j->NextSiblingElement("joint"))
		{
			const char * jn = j->Attribute("name");
			if (jn == nullptr)
			{
				continue;
			}
			bool has_cmd = false;
			double mn = 0.0, mx = 0.0, vm = 5.0;
			for (const tinyxml2::XMLElement * prm = j->FirstChildElement("param"); prm != nullptr;
				prm = prm->NextSiblingElement("param"))
			{
				const char * pn = prm->Attribute("name");
				const char * txt = prm->GetText();
				if (pn == nullptr || txt == nullptr)
				{
					continue;
				}
				if (std::strcmp(pn, "min") == 0) {mn = std::atof(txt);}
				else if (std::strcmp(pn, "max") == 0) {mx = std::atof(txt);}
				else if (std::strcmp(pn, "max_velocity") == 0) {vm = std::atof(txt);}
			}
			for (const tinyxml2::XMLElement * ci = j->FirstChildElement("command_interface"); ci != nullptr;
				ci = ci->NextSiblingElement("command_interface"))
			{
				const char * nm = ci->Attribute("name");
				if (nm != nullptr && std::strcmp(nm, "position") == 0)
				{
					has_cmd = true;
					break;
				}
			}
			if (!has_cmd)
			{
				continue;   // mimic / 无 position 命令的关节不归本控制器
			}
			if (names.size() >= unistackbot_interface::kMaxJoints)
			{
				err = "position 命令关节超过 kMaxJoints=" +
					std::to_string(unistackbot_interface::kMaxJoints) + " (当前 '" + jn + "')";
				return false;
			}
			names.push_back(jn);
			qmin.push_back(mn);
			qmax.push_back(mx);
			vmax.push_back(vm > 0.0 ? vm : 5.0);
		}
	}
	if (names.empty())
	{
		err = "URDF <ros2_control> 无 position 命令关节";
		return false;
	}
	return true;
}

}  // namespace unistackbot_controller
#endif  // UNISTACKBOT_CONTROLLER__URDF_COMMAND_JOINTS_HPP_
