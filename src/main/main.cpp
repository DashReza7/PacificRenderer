#pragma once

#include <iostream>
#include <memory>
#include "pugixml.hpp"
#include "core/Pacific.h"
#include "utils/Misc.h"
#include "core/MathUtils.h"
#include "utils/SceneParser.h"

int main()
{
	PacificParser parser;

	try {
		Scene scene = parser.parseFile("cbox.xml");

		std::cout << "Parsed scene successfully!" << std::endl;

		std::cout << scene.to_string() << std::endl;
	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
	}
}
