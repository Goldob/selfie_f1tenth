#include "process.hpp"

Process::Process()
{

}

void Process::polar_to_cartesian()
{
    cv::Point new_data;
    all_points.pos.clear();
    all_points.angle.clear();

    for(uint32_t i = raw_data.size()-1; i >= 0 ; i++)
    {
        all_points.angle.push_back(angle_min + 90 + (i * 0.36));
        new_data.x = -cos(all_points.angle[i] * (3.14159/180)) * raw_data[i];
        new_data.y = sin(all_points.angle[i] * (3.14159/180)) * raw_data[i];

        all_points.pos.push_back(new_data);
    }
}

void Process::simplify_data()
{
    cv::Point tmp, last;
    std::vector<cv::Point> tmp_vec_pos;
    std::vector<float> tmp_vec_angle;

    all_simplified.pos.clear();
    all_simplified.angle.clear();

    tmp_vec_pos.push_back(all_points.pos[0]);
    tmp_vec_angle.push_back(all_points.angle[0]);
    last = all_points.pos[0];

    for(uint32_t i = 1; i < all_points.pos.size() - 1; i++)
    {
        tmp.x = all_points.pos[i].x - last.x;
        tmp.y = all_points.pos[i].y - last.y;
        if(sqrt(tmp.x*tmp.x + tmp.y*tmp.y) > thresh_simplify)
        {
            tmp_vec_pos.push_back(all_points.pos[i]);
            tmp_vec_angle.push_back(all_points.angle[i]);
            last = all_points.pos[i];
        }
    }

    all_simplified.pos = tmp_vec_pos;
    all_simplified.angle = tmp_vec_angle;
}

void Process::split_poins_equally()
{
    bool left_continous = true;
    bool right_continous = true;

    uint32_t i = 0;
    uint32_t j = all_simplified.pos.size();

    left_points.pos.clear();
    right_points.pos.clear();
    rejected_points.pos.clear();

    left_points.angle.clear();
    right_points.angle.clear();
    rejected_points.angle.clear();

    if(all_simplified.pos.size() >= 4)
    {
        if(all_simplified.pos[all_simplified.pos.size()-1].y < 50)
        {
            left_points.pos.push_back(all_simplified.pos[all_simplified.pos.size()-1]);
        }
        else
        {
            left_continous = false;
        }

        if(all_simplified.pos[0].y < 50)
        {
            right_points.pos.push_back(all_simplified.pos[0]);
        }
        else
        {
            right_continous = false;
        }

        for(i = 1; i < all_simplified.pos.size()-1; i++)
        {
            if(left_continous)
            {
                j = all_simplified.pos.size()-1-i;
                if(sqrt((all_simplified.pos[j].x - all_simplified.pos[j+1].x)*(all_simplified.pos[j].x - all_simplified.pos[j+1].x) + (all_simplified.pos[j].y - all_simplified.pos[j+1].y)*(all_simplified.pos[j].y - all_simplified.pos[j+1].y)) < max_dist)
                    left_points.pos.push_back(all_simplified.pos[j]);
                else
                    left_continous = false;
            }

            if(right_continous)
            {
                if(sqrt((all_simplified.pos[i].x - all_simplified.pos[i-1].x)*(all_simplified.pos[i].x - all_simplified.pos[i-1].x) + (all_simplified.pos[i].y - all_simplified.pos[i-1].y)*(all_simplified.pos[i].y - all_simplified.pos[i-1].y)) < max_dist)
                    right_points.pos.push_back(all_simplified.pos[i]);
                else
                    right_continous = false;
            }

            if(all_simplified.pos.size()-1-i < i || (!left_continous && !right_continous))
                break;
        }

        int i_int = i;
        int j_int= j;

        if(j_int - i_int >= 0)
        {
            for(uint32_t k = i; k <= j; k++)
                rejected_points.pos.push_back(all_simplified.pos[k]);
        }
    }
}

void Process::search_points()
{
    left_det[0].x = left_points.pos[0].x;
    left_det[0].y = left_points.pos[0].y;
    right_det[0].x  = right_points.pos[0].x;
    right_det[0].y = right_points.pos[0].y;

    left_det[1].x = left_points.pos[left_points.pos.size()/2].x;
    left_det[1].y = left_points.pos[left_points.pos.size()/2].y;
    right_det[1].x = right_points.pos[right_points.pos.size()/2].x;
    right_det[1].y = right_points.pos[right_points.pos.size()/2].y;


    left_det[DET_NUM-1].x = left_points.pos[left_points.pos.size()-1].x;
    left_det[DET_NUM-1].y = left_points.pos[left_points.pos.size()-1].y;
    right_det[DET_NUM-1].x = right_points.pos[right_points.pos.size()-1].x;
    right_det[DET_NUM-1].y = right_points.pos[right_points.pos.size()-1].y;

}

void Process::calc_mid()
{
    for(uint32_t i = 0; i < DET_NUM; i++)
    {
        mid_det[i].x = (left_det[i].x + right_det[i].x)/2;
        mid_det[i].y = (left_det[i].y + right_det[i].y)/2;
    }
}

void Process::calc_offsets()
{
    for(uint32_t i = 0; i < DET_NUM; i++)
        offset[i] = mid_det[i].x - LIDAR_POS_X;
}

void Process::calc_slopes()
{
    for(uint32_t i = 0; i < DET_NUM-1; i++)
        slope[i] = (float)(mid_det[i+1].y - mid_det[i].y)/(float)(mid_det[i].x - mid_det[i+1].x);
}

void Process::filter_enemies()
{
    int Px, Py, alpha;
    int Vx = left_det[DET_NUM-1].x - right_det[DET_NUM-1].x;
    int Vy = left_det[DET_NUM-1].y - right_det[DET_NUM-1].y;

    enemies_points.pos.clear();
    trash_points.pos.clear();

    for(uint32_t i = 0; i < rejected_points.pos.size(); i++)
    {
        Px = rejected_points.pos[i].x - right_det[DET_NUM-1].x;
        Py = rejected_points.pos[i].y - right_det[DET_NUM-1].y;

        alpha = Vx*Py - Vy*Px;

        if(alpha < 0)
            enemies_points.pos.push_back(rejected_points.pos[i]);
        else
            trash_points.pos.push_back(rejected_points.pos[i]);
    }
}

void Process::pack_data(SteerData &out)
{
    out.offset[0] = mid_det[0].x;
    out.offset[1] = mid_det[1].x;
    out.offset[DET_NUM-1] = mid_det[DET_NUM-1].x;

    out.slope[0] = slope[0];
    out.slope[DET_NUM-2] = slope[DET_NUM-2];
}

void Process::display_data()
{
    std::cout << "\033[2J\033[1;1H";
    std::cout << "MID 3 offset: " << offset[2] << std::endl;
    std::cout << "MID 2 offset: " << offset[1] << std::endl;
    std::cout << "MID 1 offset: " << offset[0] << std::endl;
    std::cout << "STAGE 2 angle: " << slope[1] << std::endl;
    std::cout << "STAGE 1 angle: " << slope[0] << std::endl;
}
