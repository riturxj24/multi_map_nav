#include "multi_map_nav/wormhole_db.hpp"
#include <iostream>
#include <sstream>
#include <queue>
#include <map>

namespace multi_map_nav
{

WormholeDB::WormholeDB()
: db_(nullptr), logger_(rclcpp::get_logger("wormhole_db"))
{
}

WormholeDB::~WormholeDB()
{
  close();
}

bool WormholeDB::initialize(const std::string &db_path)
{
  RCLCPP_INFO(logger_, "Initializing wormhole database: %s", db_path.c_str());
  
  
  int rc = sqlite3_open(db_path.c_str(), &db_);
  if (rc != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to open database: %s", sqlite3_errmsg(db_));
    sqlite3_close(db_);
    db_ = nullptr;
    return false;
  }
  
  
  if (!create_tables()) {
    RCLCPP_ERROR(logger_, "Failed to create database tables");
    close();
    return false;
  }
  
  RCLCPP_INFO(logger_, "Wormhole database initialized successfully");
  return true;
}

bool WormholeDB::create_tables()
{
  const char* wormholes_table = 
    "CREATE TABLE IF NOT EXISTS wormholes ("
    "id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "source_map TEXT NOT NULL,"
    "target_map TEXT NOT NULL,"
    "source_x REAL NOT NULL,"
    "source_y REAL NOT NULL,"
    "source_z REAL NOT NULL,"
    "source_qx REAL NOT NULL,"
    "source_qy REAL NOT NULL,"
    "source_qz REAL NOT NULL,"
    "source_qw REAL NOT NULL,"
    "target_x REAL NOT NULL,"
    "target_y REAL NOT NULL,"
    "target_z REAL NOT NULL,"
    "target_qx REAL NOT NULL,"
    "target_qy REAL NOT NULL,"
    "target_qz REAL NOT NULL,"
    "target_qw REAL NOT NULL,"
    "transition_cost REAL NOT NULL,"
    "UNIQUE(source_map, target_map));";

  char* err_msg = nullptr;
  
  
  if (sqlite3_exec(db_, wormholes_table, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "SQL error creating wormholes table: %s", err_msg);
    sqlite3_free(err_msg);
    return false;
  }
  
  return true;
}


bool WormholeDB::create_trajectory_table()
{
  const char* trajectory_table =
    "CREATE TABLE IF NOT EXISTS robot_trajectories ("
    "id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "map_id TEXT NOT NULL,"
    "timestamp INTEGER NOT NULL,"
    "pose_x REAL NOT NULL,"
    "pose_y REAL NOT NULL,"
    "pose_z REAL NOT NULL,"
    "pose_qx REAL NOT NULL,"
    "pose_qy REAL NOT NULL,"
    "pose_qz REAL NOT NULL,"
    "pose_qw REAL NOT NULL);";

  char* err_msg = nullptr;
  if (sqlite3_exec(db_, trajectory_table, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "SQL error creating trajectory table: %s", err_msg);
    sqlite3_free(err_msg);
    return false;
  }
  return true;
}


void WormholeDB::close()
{
  if (db_) {
    sqlite3_close(db_);
    db_ = nullptr;
    RCLCPP_INFO(logger_, "Wormhole database closed");
  }
}

bool WormholeDB::add_wormhole(const Wormhole& wormhole)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }
  
  const char* sql = 
    "INSERT OR REPLACE INTO wormholes "
    "(source_map, target_map, source_x, source_y, source_z, source_qx, source_qy, source_qz, source_qw, "
    "target_x, target_y, target_z, target_qx, target_qy, target_qz, target_qw, transition_cost) "
    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";
  
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement: %s", sqlite3_errmsg(db_));
    return false;
  }
  
  sqlite3_bind_text(stmt, 1, wormhole.source_map.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_text(stmt, 2, wormhole.target_map.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_double(stmt, 3, wormhole.source_pose.position.x);
  sqlite3_bind_double(stmt, 4, wormhole.source_pose.position.y);
  sqlite3_bind_double(stmt, 5, wormhole.source_pose.position.z);
  sqlite3_bind_double(stmt, 6, wormhole.source_pose.orientation.x);
  sqlite3_bind_double(stmt, 7, wormhole.source_pose.orientation.y);
  sqlite3_bind_double(stmt, 8, wormhole.source_pose.orientation.z);
  sqlite3_bind_double(stmt, 9, wormhole.source_pose.orientation.w);
  sqlite3_bind_double(stmt, 10, wormhole.target_pose.position.x);
  sqlite3_bind_double(stmt, 11, wormhole.target_pose.position.y);
  sqlite3_bind_double(stmt, 12, wormhole.target_pose.position.z);
  sqlite3_bind_double(stmt, 13, wormhole.target_pose.orientation.x);
  sqlite3_bind_double(stmt, 14, wormhole.target_pose.orientation.y);
  sqlite3_bind_double(stmt, 15, wormhole.target_pose.orientation.z);
  sqlite3_bind_double(stmt, 16, wormhole.target_pose.orientation.w);
  sqlite3_bind_double(stmt, 17, wormhole.transition_cost);
  
  bool success = sqlite3_step(stmt) == SQLITE_DONE;
  if (!success) {
    RCLCPP_ERROR(logger_, "Failed to add wormhole: %s", sqlite3_errmsg(db_));
  }
  
  sqlite3_finalize(stmt);
  return success;
}

bool WormholeDB::remove_wormhole(const std::string& source_map, const std::string& target_map)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }
  
  const char* sql = 
    "DELETE FROM wormholes WHERE source_map = ? AND target_map = ?;";
  
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement: %s", sqlite3_errmsg(db_));
    return false;
  }
  
  sqlite3_bind_text(stmt, 1, source_map.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_text(stmt, 2, target_map.c_str(), -1, SQLITE_STATIC);
  
  bool success = sqlite3_step(stmt) == SQLITE_DONE;
  if (!success) {
    RCLCPP_ERROR(logger_, "Failed to remove wormhole: %s", sqlite3_errmsg(db_));
  }
  
  sqlite3_finalize(stmt);
  return success;
}

bool WormholeDB::get_wormholes(const std::string& map_name, std::vector<Wormhole>& wormholes)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }
  
  const char* sql = 
    "SELECT source_map, target_map, "
    "source_x, source_y, source_z, source_qx, source_qy, source_qz, source_qw, "
    "target_x, target_y, target_z, target_qx, target_qy, target_qz, target_qw, "
    "transition_cost "
    "FROM wormholes WHERE source_map = ?;";
  
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement: %s", sqlite3_errmsg(db_));
    return false;
  }
  
  sqlite3_bind_text(stmt, 1, map_name.c_str(), -1, SQLITE_STATIC);
  
  wormholes.clear();
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    Wormhole wormhole;
    wormhole.source_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
    wormhole.target_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
    
    wormhole.source_pose.position.x = sqlite3_column_double(stmt, 2);
    wormhole.source_pose.position.y = sqlite3_column_double(stmt, 3);
    wormhole.source_pose.position.z = sqlite3_column_double(stmt, 4);
    wormhole.source_pose.orientation.x = sqlite3_column_double(stmt, 5);
    wormhole.source_pose.orientation.y = sqlite3_column_double(stmt, 6);
    wormhole.source_pose.orientation.z = sqlite3_column_double(stmt, 7);
    wormhole.source_pose.orientation.w = sqlite3_column_double(stmt, 8);
    
    wormhole.target_pose.position.x = sqlite3_column_double(stmt, 9);
    wormhole.target_pose.position.y = sqlite3_column_double(stmt, 10);
    wormhole.target_pose.position.z = sqlite3_column_double(stmt, 11);
    wormhole.target_pose.orientation.x = sqlite3_column_double(stmt, 12);
    wormhole.target_pose.orientation.y = sqlite3_column_double(stmt, 13);
    wormhole.target_pose.orientation.z = sqlite3_column_double(stmt, 14);
    wormhole.target_pose.orientation.w = sqlite3_column_double(stmt, 15);
    
    wormhole.transition_cost = sqlite3_column_double(stmt, 16);
    
    wormholes.push_back(wormhole);
  }
  
  sqlite3_finalize(stmt);
  return true;
}

bool WormholeDB::find_path(const std::string& start_map, const geometry_msgs::msg::Pose& /*start_pose*/,
                           const std::string& goal_map, const geometry_msgs::msg::Pose& /*goal_pose*/,
                           std::vector<Wormhole>& path)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }
  
  path.clear();
  
  
  if (start_map == goal_map) {
    return true;
  }
  
  
  const char* direct_sql = 
    "SELECT source_map, target_map, "
    "source_x, source_y, source_z, source_qx, source_qy, source_qz, source_qw, "
    "target_x, target_y, target_z, target_qx, target_qy, target_qz, target_qw, "
    "transition_cost "
    "FROM wormholes WHERE source_map = ? AND target_map = ?;";
  
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db_, direct_sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement: %s", sqlite3_errmsg(db_));
    return false;
  }
  
  sqlite3_bind_text(stmt, 1, start_map.c_str(), -1, SQLITE_STATIC);
  sqlite3_bind_text(stmt, 2, goal_map.c_str(), -1, SQLITE_STATIC);
  
  
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    Wormhole wormhole;
    wormhole.source_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
    wormhole.target_map = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
    
    wormhole.source_pose.position.x = sqlite3_column_double(stmt, 2);
    wormhole.source_pose.position.y = sqlite3_column_double(stmt, 3);
    wormhole.source_pose.position.z = sqlite3_column_double(stmt, 4);
    wormhole.source_pose.orientation.x = sqlite3_column_double(stmt, 5);
    wormhole.source_pose.orientation.y = sqlite3_column_double(stmt, 6);
    wormhole.source_pose.orientation.z = sqlite3_column_double(stmt, 7);
    wormhole.source_pose.orientation.w = sqlite3_column_double(stmt, 8);
    
    wormhole.target_pose.position.x = sqlite3_column_double(stmt, 9);
    wormhole.target_pose.position.y = sqlite3_column_double(stmt, 10);
    wormhole.target_pose.position.z = sqlite3_column_double(stmt, 11);
    wormhole.target_pose.orientation.x = sqlite3_column_double(stmt, 12);
    wormhole.target_pose.orientation.y = sqlite3_column_double(stmt, 13);
    wormhole.target_pose.orientation.z = sqlite3_column_double(stmt, 14);
    wormhole.target_pose.orientation.w = sqlite3_column_double(stmt, 15);
    
    wormhole.transition_cost = sqlite3_column_double(stmt, 16);
    
    path.push_back(wormhole);
    sqlite3_finalize(stmt);
    return true;
  }
  
  sqlite3_finalize(stmt);
  
 
  std::map<std::string, std::string> visited;
  visited[start_map] = "";  // Start map has no predecessor
  
  
  std::queue<std::string> queue;
  queue.push(start_map);
  

  std::map<std::string, Wormhole> edges;
  

  bool found_path = false;
  while (!queue.empty() && !found_path) {
    std::string current_map = queue.front();
    queue.pop();
    

    std::vector<Wormhole> outgoing_wormholes;
    get_wormholes(current_map, outgoing_wormholes);
    
    for (const auto& wormhole : outgoing_wormholes) {
      const std::string& next_map = wormhole.target_map;
      

      std::string edge_key = wormhole.source_map + "->" + wormhole.target_map;
      edges[edge_key] = wormhole;
      

      if (next_map == goal_map) {
        visited[next_map] = current_map;
        found_path = true;
        break;
      }
      

      if (visited.find(next_map) == visited.end()) {
        visited[next_map] = current_map;
        queue.push(next_map);
      }
    }
  }
  

  if (found_path) {
    std::vector<Wormhole> reverse_path;
    std::string current = goal_map;
    

    while (current != start_map) {
      std::string previous = visited[current];
      std::string edge_key = previous + "->" + current;
      reverse_path.push_back(edges[edge_key]);
      current = previous;
    }
    

    for (auto it = reverse_path.rbegin(); it != reverse_path.rend(); ++it) {
      path.push_back(*it);
    }
    
    RCLCPP_INFO(logger_, "Found multi-hop path with %zu wormholes", path.size());
    return true;
  }
  
  RCLCPP_ERROR(logger_, "No path found between %s and %s", start_map.c_str(), goal_map.c_str());
  return false;
}


bool WormholeDB::store_trajectory(
  const std::string& map_id,
  const std::vector<geometry_msgs::msg::Pose>& trajectory,
  const std::chrono::system_clock::time_point& timestamp)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }


  if (!create_trajectory_table()) {
      RCLCPP_ERROR(logger_, "Failed to ensure trajectory table exists.");
      return false;
  }
  
  const char* sql =
    "INSERT INTO robot_trajectories "
    "(map_id, timestamp, pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw) "
    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?);";
  
  sqlite3_stmt* stmt;
  int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement for trajectory storage: %s", sqlite3_errmsg(db_));
    return false;
  }


  long long timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    timestamp.time_since_epoch()).count();
  
  bool success = true;
  for (const auto& pose : trajectory) {
    sqlite3_reset(stmt); // Reset for new binding
    sqlite3_bind_text(stmt, 1, map_id.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_int64(stmt, 2, timestamp_ms);
    sqlite3_bind_double(stmt, 3, pose.position.x);
    sqlite3_bind_double(stmt, 4, pose.position.y);
    sqlite3_bind_double(stmt, 5, pose.position.z);
    sqlite3_bind_double(stmt, 6, pose.orientation.x);
    sqlite3_bind_double(stmt, 7, pose.orientation.y);
    sqlite3_bind_double(stmt, 8, pose.orientation.z);
    sqlite3_bind_double(stmt, 9, pose.orientation.w);
    
    if (sqlite3_step(stmt) != SQLITE_DONE) {
      RCLCPP_ERROR(logger_, "Failed to store trajectory point: %s", sqlite3_errmsg(db_));
      success = false;
      break;
    }
  }
  
  sqlite3_finalize(stmt);
  return success;
}

bool WormholeDB::get_trajectories(
  const std::string& map_id,
  std::map<std::string, std::vector<geometry_msgs::msg::Pose>>& trajectories,
  const std::chrono::system_clock::time_point* start_time,
  const std::chrono::system_clock::time_point* end_time)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }

  trajectories.clear(); // Clear existing data

  std::stringstream sql_stream;
  sql_stream << "SELECT map_id, pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw, timestamp FROM robot_trajectories";
  
  std::vector<std::string> conditions;
  std::vector<long long> time_bind_values;

  if (!map_id.empty()) {
    conditions.push_back("map_id = ?");
  }
  
  if (start_time) {
    conditions.push_back("timestamp >= ?");
    time_bind_values.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(start_time->time_since_epoch()).count());
  }
  
  if (end_time) {
    conditions.push_back("timestamp <= ?");
    time_bind_values.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(end_time->time_since_epoch()).count());
  }

  if (!conditions.empty()) {
    sql_stream << " WHERE " << conditions[0];
    for (size_t i = 1; i < conditions.size(); ++i) {
      sql_stream << " AND " << conditions[i];
    }
  }
  sql_stream << " ORDER BY map_id, timestamp;";

  std::string sql = sql_stream.str();
  
  sqlite3_stmt* stmt;
  int rc = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement for trajectory retrieval: %s", sqlite3_errmsg(db_));
    return false;
  }

  int bind_idx = 1;
  if (!map_id.empty()) {
    sqlite3_bind_text(stmt, bind_idx++, map_id.c_str(), -1, SQLITE_STATIC);
  }
  for (long long val : time_bind_values) {
    sqlite3_bind_int64(stmt, bind_idx++, val);
  }
  
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    std::string current_map_id = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
    geometry_msgs::msg::Pose pose;
    pose.position.x = sqlite3_column_double(stmt, 1);
    pose.position.y = sqlite3_column_double(stmt, 2);
    pose.position.z = sqlite3_column_double(stmt, 3);
    pose.orientation.x = sqlite3_column_double(stmt, 4);
    pose.orientation.y = sqlite3_column_double(stmt, 5);
    pose.orientation.z = sqlite3_column_double(stmt, 6);
    pose.orientation.w = sqlite3_column_double(stmt, 7);
    
    trajectories[current_map_id].push_back(pose);
  }
  
  sqlite3_finalize(stmt);
  return true;
}


float WormholeDB::calculate_path_cost(const std::vector<Wormhole>& path) const
{
  float total_cost = 0.0;
  for (const auto& wormhole : path) {
    total_cost += wormhole.transition_cost;
  }
  RCLCPP_INFO(logger_, "Calculated path cost: %f", total_cost);
  return total_cost;
}

bool WormholeDB::get_connected_maps(const std::string& map_name, std::vector<std::string>& connected_maps)
{
  if (!db_) {
    RCLCPP_ERROR(logger_, "Database not initialized");
    return false;
  }

  connected_maps.clear();
  const char* sql = "SELECT DISTINCT target_map FROM wormholes WHERE source_map = ?;";
  sqlite3_stmt* stmt;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "Failed to prepare statement: %s", sqlite3_errmsg(db_));
    return false;
  }

  sqlite3_bind_text(stmt, 1, map_name.c_str(), -1, SQLITE_STATIC);

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    connected_maps.push_back(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
  }

  sqlite3_finalize(stmt);
  return true;
}


}  
