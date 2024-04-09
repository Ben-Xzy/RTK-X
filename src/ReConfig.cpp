#include "ReConfig.h"
#include <fstream>
#include <stdlib.h>
#include"DataClassSet.h"

namespace rr
{

	bool RrConfig::IsSpace(char c)
	{
		if (' ' == c || '\t' == c)
			return true;
		return false;
	}

	bool RrConfig::IsCommentChar(char c)
	{
		switch (c) {
		case '#':
			return true;
		default:
			return false;
		}
	}

	void RrConfig::Trim(std::string& str)
	{
		if (str.empty())
		{
			return;
		}
		int i, start_pos, end_pos;
		for (i = 0; i < str.size(); ++i) 
		{
			if (!IsSpace(str[i])) {
				break;
			}
		}
		if (i == str.size())
		{
			str = "";
			return;
		}
		start_pos = i;
		for (i = str.size() - 1; i >= 0; --i) {
			if (!IsSpace(str[i])) {
				break;
			}
		}
		end_pos = i;
		str = str.substr(start_pos, end_pos - start_pos + 1);
	}

	bool RrConfig::AnalyseLine(const std::string& line, std::string& section, std::string& key, std::string& value)
	{
		if (line.empty())
			return false;
		int start_pos = 0, end_pos = line.size() - 1, pos, s_startpos, s_endpos;
		if ((pos = line.find("#")) != -1)/*说明不是注释*/
		{
			if (0 == pos)
			{
				return false;
			}
			end_pos = pos - 1;
		}
		if (((s_startpos = line.find("[")) != -1) && ((s_endpos = line.find("]"))) != -1)
		{
			section = line.substr(s_startpos + 1, s_endpos - 1);
			return true;
		}
		std::string new_line = line.substr(start_pos, start_pos + 1 - end_pos);
		if ((pos = new_line.find('=')) == -1)
			return false;
		key = new_line.substr(0, pos);
		value = new_line.substr(pos + 1, end_pos + 1 - (pos + 1));
		Trim(key);
		if (key.empty()) {
			return false;
		}
		Trim(value);
		if ((pos = value.find("\r")) > 0)/*换行符*/
		{
			value.replace(pos, 1, "");
		}
		if ((pos = value.find("\n")) > 0)
		{
			value.replace(pos, 1, "");
		}
		return true;
	}

	bool RrConfig::ReadConfig(const std::string& filename)
	{
		settings_.clear();
		std::ifstream infile(filename.c_str());//构造默认调用open,所以可以不调用open
		//std::ifstream infile;
		//infile.open(filename.c_str());
		//bool ret = infile.is_open()
		if (!infile) {
			return false;
		}
		std::string line, key, value, section;
		std::map<std::string, std::string> k_v;
		std::map<std::string, std::map<std::string, std::string> >::iterator it;/*map的find返回的是迭代器*/
		while (getline(infile, line))
		{
			if (AnalyseLine(line, section, key, value))
			{
				it = settings_.find(section);/*C++中map未找到条目时返回末尾值*/
				if (it != settings_.end())
				{
					k_v[key] = value;
					it->second = k_v;
				}
				else
				{
					k_v.clear();
					settings_.insert(std::make_pair(section, k_v));
				}
			}
			key.clear();
			value.clear();
		}
		infile.close();
		return true;
	}

	std::string RrConfig::ReadString(const char* section, const char* item, const char* default_value)
	{
		std::string tmp_s(section);
		std::string tmp_i(item);
		std::string def(default_value);
		std::map<std::string, std::string> k_v;
		std::map<std::string, std::string>::iterator it_item;
		std::map<std::string, std::map<std::string, std::string> >::iterator it;
		it = settings_.find(tmp_s);
		if (it == settings_.end())
		{
			//printf("111");
			return def;
		}
		k_v = it->second;
		it_item = k_v.find(tmp_i);
		if (it_item == k_v.end())
		{
			//printf("222");
			return def;
		}
		return it_item->second;
	}

	int RrConfig::ReadInt(const char* section, const char* item, const int& default_value)
	{
		std::string tmp_s(section);
		std::string tmp_i(item);
		std::map<std::string, std::string> k_v;
		std::map<std::string, std::string>::iterator it_item;
		std::map<std::string, std::map<std::string, std::string> >::iterator it;
		it = settings_.find(tmp_s);
		if (it == settings_.end())
		{
			return default_value;
		}
		k_v = it->second;
		it_item = k_v.find(tmp_i);
		if (it_item == k_v.end())
		{
			return default_value;
		}
		return atoi(it_item->second.c_str());
	}

	float RrConfig::ReadFloat(const char* section, const char* item, const float& default_value)
	{
		std::string tmp_s(section);
		std::string tmp_i(item);
		std::map<std::string, std::string> k_v;
		std::map<std::string, std::string>::iterator it_item;
		std::map<std::string, std::map<std::string, std::string> >::iterator it;
		it = settings_.find(tmp_s);
		if (it == settings_.end())
		{
			return default_value;
		}
		k_v = it->second;
		it_item = k_v.find(tmp_i);
		if (it_item == k_v.end())
		{
			return default_value;
		}
		return atof(it_item->second.c_str());
	}
	double RrConfig::ReadDouble(const char* section, const char* item, const double& default_value)
{
	std::string tmp_s(section);
	std::string tmp_i(item);
	std::map<std::string, std::string> k_v;
	std::map<std::string, std::string>::iterator it_item;
	std::map<std::string, std::map<std::string, std::string> >::iterator it;
	it = settings_.find(tmp_s);
	if (it == settings_.end())
	{
		return default_value;
	}
	k_v = it->second;
	it_item = k_v.find(tmp_i);
	if (it_item == k_v.end())
	{
		return default_value;
	}
	return std::stod(it_item->second.c_str());
}
	bool GetCfgInfo(ROVERCFGINFO& CFG, std::string filename)
		{
			rr::RrConfig config;
			if (!config.ReadConfig(filename))
			{
				return false;
			}
			CFG.AmbNoise = config.ReadDouble("EKF", "AmbRErr", 0.0);
			CFG.AmbPIni = config.ReadDouble("EKF", "AmbPErr", 0.0);
			CFG.AmbQErr = config.ReadDouble("EKF", "AmbQErr", 0.0);
			CFG.BasNetIP = config.ReadString("socket", "BasIp", "null");
			CFG.BasNetPort = config.ReadInt("socket", "BasPort", 0);
			CFG.BasObsDatFile = config.ReadString("file", "BasFilePath", "null");
			CFG.CPNoise = config.ReadDouble("EKF", "LRIni", 0.0);
			CFG.CodeNoise = config.ReadDouble("EKF", "PRIni", 0.0);
			CFG.DisModel = config.ReadString("model", "DispModel", "null");
			CFG.ElevThreshold = config.ReadDouble("threshold", "Ema", 0.0);
			std::istringstream(config.ReadString("model", "EmaFlag", "0")) >> CFG.EmaFlag;
			CFG.IsFileData = config.ReadString("model", "Model", "socket");
			CFG.PosPIni = config.ReadDouble("EKF", "PosPErr", 0.0);
			CFG.PosQErr = config.ReadDouble("EKF", "PosQErr", 0.0);
			CFG.ResFile = config.ReadString("file", "ResFilePath", "null");
			CFG.RovNetIP = config.ReadString("socket", "RovIp", "null");
			CFG.RovNetPort = config.ReadInt("socket", "RovPort", 0);
			CFG.RovObsDatFile = config.ReadString("file", "RovFilePath", "null");
			CFG.RTKProcMode = config.ReadString("model", "CalModel", "null");
			CFG.RtkSynFileThre = config.ReadDouble("threshold", "RtkSynFileThre", 0.0);
			CFG.RtkSynSktThre = config.ReadDouble("threshold", "RtkSynSktThre", 0.0);
			return true;
		}
}


