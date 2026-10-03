/**
 * @file macro_library.cpp
 * @brief Loading the macro directory and merging its models into a palette.
 */

#include "g1_orchestration/macro_library.hpp"

#include <tinyxml2.h>

#include <algorithm>
#include <filesystem>
#include <format>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "g1_orchestration/xml_canonical.hpp"

namespace g1_orchestration
{

namespace
{

[[noreturn]] void fail(const std::filesystem::path& file, const std::string& what)
{
    throw std::runtime_error(std::format("macro library: {}: {}", file.filename().string(), what));
}

std::string idOf(const tinyxml2::XMLElement& element, const std::filesystem::path& file)
{
    const char* id = element.Attribute("ID");
    if (id == nullptr || *id == '\0')
    {
        fail(file, std::format("<{}> has no ID", element.Name()));
    }
    return id;
}

std::vector<std::filesystem::path> xmlFilesIn(const std::filesystem::path& dir)
{
    if (!std::filesystem::is_directory(dir))
    {
        throw std::runtime_error("macro library: " + dir.string() + " is not a directory");
    }
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(dir))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".xml")
        {
            files.push_back(entry.path());
        }
    }
    std::ranges::sort(files);
    return files;
}

std::string readText(const std::filesystem::path& file)
{
    std::ifstream in(file);
    if (!in)
    {
        fail(file, "cannot be read");
    }
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

void readFile(const std::filesystem::path& file, std::vector<Macro>& into)
{
    const std::string     text = readText(file);
    tinyxml2::XMLDocument doc;
    if (doc.Parse(text.c_str(), text.size()) != tinyxml2::XML_SUCCESS)
    {
        fail(file, doc.ErrorStr());
    }
    const tinyxml2::XMLElement* root = doc.RootElement();
    if (root == nullptr || std::string_view(root->Name()) != "root")
    {
        fail(file, "the root element must be <root>");
    }

    std::vector<Macro> trees;
    std::vector<Macro> models;
    for (const tinyxml2::XMLElement* child = root->FirstChildElement(); child != nullptr;
         child                             = child->NextSiblingElement())
    {
        const std::string_view tag = child->Name();
        if (tag == "BehaviorTree")
        {
            Macro macro;
            macro.id   = idOf(*child, file);
            macro.xml  = text;
            macro.tags = tagsUnder(*child);
            macro.tags.erase(macro.tags.begin());
            if (std::ranges::find(macro.tags, "SubTree") != macro.tags.end())
            {
                fail(file, macro.id + " names another macro; macros cannot nest");
            }
            trees.push_back(std::move(macro));
        }
        else if (tag == "TreeNodesModel")
        {
            for (const tinyxml2::XMLElement* entry = child->FirstChildElement("SubTree");
                 entry != nullptr;
                 entry = entry->NextSiblingElement("SubTree"))
            {
                Macro model;
                model.id    = idOf(*entry, file);
                model.model = canonicalXml(*entry);
                for (const tinyxml2::XMLElement* port = entry->FirstChildElement("input_port");
                     port != nullptr;
                     port = port->NextSiblingElement("input_port"))
                {
                    const char* port_name = port->Attribute("name");
                    model.ports.emplace_back(port_name != nullptr ? port_name : "");
                }
                models.push_back(std::move(model));
            }
        }
        else
        {
            fail(file, std::format("unexpected <{}> in <root>", tag));
        }
    }

    for (const Macro& model : models)
    {
        if (std::ranges::none_of(trees, [&model](const Macro& m) { return m.id == model.id; }))
        {
            fail(
                file,
                "<TreeNodesModel> describes " + model.id + ", which this file does not define");
        }
    }
    for (Macro& macro : trees)
    {
        const auto model = std::ranges::find(models, macro.id, &Macro::id);
        if (model == models.end())
        {
            fail(file, macro.id + " has no <SubTree> entry in <TreeNodesModel>");
        }
        macro.model = model->model;
        macro.ports = model->ports;
        into.push_back(std::move(macro));
    }
}

}  // namespace

MacroLibrary MacroLibrary::load(const std::filesystem::path& dir)
{
    MacroLibrary library;
    for (const std::filesystem::path& file : xmlFilesIn(dir))
    {
        readFile(file, library.macros_);
    }
    for (std::size_t i = 0; i < library.macros_.size(); ++i)
    {
        for (std::size_t j = i + 1; j < library.macros_.size(); ++j)
        {
            if (library.macros_[i].id == library.macros_[j].id)
            {
                throw std::runtime_error(
                    "macro library: " + library.macros_[i].id + " defined twice");
            }
        }
    }
    return library;
}

const Macro* MacroLibrary::find(std::string_view id) const
{
    const auto found = std::ranges::find(macros_, id, &Macro::id);
    return found == macros_.end() ? nullptr : &*found;
}

std::string MacroLibrary::withModelsIn(const std::string& palette_xml) const
{
    tinyxml2::XMLDocument palette;
    if (palette.Parse(palette_xml.c_str(), palette_xml.size()) != tinyxml2::XML_SUCCESS)
    {
        throw std::runtime_error(std::string("palette is not XML: ") + palette.ErrorStr());
    }
    tinyxml2::XMLElement* model_root =
        palette.RootElement() == nullptr ?
            nullptr :
            palette.RootElement()->FirstChildElement("TreeNodesModel");
    if (model_root == nullptr)
    {
        throw std::runtime_error("palette has no <TreeNodesModel>");
    }
    for (const Macro& macro : macros_)
    {
        tinyxml2::XMLDocument entry;
        entry.Parse(macro.model.c_str(), macro.model.size());
        model_root->InsertEndChild(entry.RootElement()->DeepClone(&palette));
    }
    tinyxml2::XMLPrinter printer;
    palette.Print(&printer);
    return printer.CStr();
}

}  // namespace g1_orchestration
