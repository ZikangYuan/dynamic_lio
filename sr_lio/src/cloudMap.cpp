#include "cloudMap.h"

globalPoint::globalPoint()
{

}

void globalPoint::setPosition(Eigen::Vector3d &position_)
{
	position = position_.cast<float>();
}

void globalPoint::setLabel(int label_)
{
	label = (short)label_;
}

void globalPoint::setDynamic(bool is_dynamic_)
{
	is_dynamic = is_dynamic_;
}

Eigen::Vector3d globalPoint::getPosition()
{
	return position.cast<double>();
}

int globalPoint::getLabel()
{
	return (int)label;
}

bool globalPoint::isDynamic()
{
	return is_dynamic;
}