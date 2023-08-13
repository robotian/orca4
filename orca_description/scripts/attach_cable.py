import xml.etree.ElementTree as ET
import math
# import re
import sys


linkType = ['pitch','yaw','body']


def add_pose2parent(parentEl, elStr, relative_to=None):
    if relative_to==None:
        el = ET.Element('pose')
    else:
        el = ET.Element('pose',{'relative_to':relative_to})
    el.text = elStr
    parentEl.append(el)

def add_mass2parent(parentEl, elStr):
    el = ET.Element('mass')
    el.text = elStr
    parentEl.append(el)

def add_inertia2parent(parentEl, ixx, ixy, ixz, iyy, iyz, izz):
    el = ET.Element('inertia')
    subEl = ET.Element('ixx')
    subEl.text = ixx
    el.append(subEl)
    subEl = ET.Element('ixy')
    subEl.text = ixy
    el.append(subEl)
    subEl = ET.Element('ixz')
    subEl.text = ixz
    el.append(subEl)
    subEl = ET.Element('iyy')
    subEl.text = iyy
    el.append(subEl)
    subEl = ET.Element('iyz')
    subEl.text = iyz
    el.append(subEl)
    subEl = ET.Element('izz')
    subEl.text = izz
    el.append(subEl)
    parentEl.append(el)


def add_inertial2parent(parentEl, massStr, ixx, ixy, ixz, iyy, iyz, izz):
    el = ET.Element('inertial')
    add_mass2parent(el,massStr)    
    add_inertia2parent(el, ixx, ixy, ixz, iyy, iyz, izz)
    parentEl.append(el)

def add_tagwithText(parentEl, tagName, textStr):
    el = ET.Element(tagName)
    el.text = textStr
    parentEl.append(el)


def add_cylinder2parent(parentEl, len, rad):
    el = ET.Element('geometry')
    
    el2 = ET.Element('cylinder')
    add_tagwithText(el2,'length',len)
    add_tagwithText(el2,'radius',rad)
    el.append(el2)
    parentEl.append(el)


def add_visual2parent(parentEl, tagName, visPose, geoType, var1, var2):
    el = ET.Element('visual',{'name':tagName})
    add_pose2parent(el,visPose)

    #cylinder
    if geoType == 'cylinder':
        add_cylinder2parent(el, var1, var2)

    
    parentEl.append(el)

def add_collision2parent(parentEl, tagName, visPose, geoType, var1, var2):
    el = ET.Element('collision',{'name':tagName})
    add_pose2parent(el,visPose)

    #cylinder
    if geoType == 'cylinder':
        add_cylinder2parent(el, var1, var2)

    
    parentEl.append(el)


def add_axis2parent(parentEl, expressed_in, xyz):
    el = ET.Element('axis')

    el1 = ET.Element('xyz',{'expressed_in':expressed_in})
    el1.text = xyz
    el.append(el1)

    el1 = ET.Element('limit')
    add_tagwithText(el1,'lower','-1.5707963267948966')
    add_tagwithText(el1,'upper','1.5707963267948966')    
    el.append(el1)

    parentEl.append(el)


small_mass = str(1.0e-10)
small_inertia = str(1.0e-10)
link_mass = str(1.0e-5)
link_iner = str(1.0e-5)

def add_cable_seg(input_path, output_path):    
    tree = ET.parse(input_path)

    root = tree.getroot()
    
    subEl = ET.SubElement(root[0],'link',attrib={'name':'cable_base_link'})
    add_pose2parent(subEl,'-0.2 0 0 0 0 0','base_link')
    add_inertial2parent(subEl,small_mass,small_inertia,'0','0',small_inertia,'0',small_inertia)
 
    subEl = ET.SubElement(root[0],'joint',attrib={'name':'cable_base_jnt','type':'fixed'})
    add_tagwithText(subEl,'parent','base_link')
    add_tagwithText(subEl,'child','cable_base_link')

    
    preLink = 'cable_base_link'
    for segNum in range(1,51):
        for lt in linkType:
            # if lt in ['pitch','yaw']:
            lnName = 'seg'+str(segNum)+'_'+lt
            jntName = lnName + '_jnt'
            visName = lnName + '_vis'
            colName = lnName + '_col'

            if lt == 'pitch':
                if segNum == 1:
                    link_pose = '0 0 0 0 0 0'
                else:
                    link_pose = '-0.2 0 0 0 0 0'
                vis_pose = '0 0 0 1.5707963267948966  0 0'
                cylinder_len = '0.02'
                cylinder_rad = '0.01'
                jnt_axis = '0 1 0'
                jnt_pose = '0 0 0 0 0 0'
                link_inertia = [small_mass,small_inertia,'0','0',small_inertia,'0',small_inertia]
            elif lt == 'yaw':
                link_pose = '0 0 0 0 0 0'
                vis_pose = '0 0 0 0 0 0'
                cylinder_len = '0.02'
                cylinder_rad = '0.01'
                jnt_axis = '0 0 1'
                jnt_pose = '0 0 0 0 0 0'
                link_inertia = [small_mass,small_inertia,'0','0',small_inertia,'0',small_inertia]
            else:
                link_pose = '-0.2 0 0 0 0 0'
                vis_pose = '0 0 0 0 1.5707963267948966 0'
                cylinder_len = '0.4'
                cylinder_rad = '0.01'
                jnt_axis = '1 0 0'
                jnt_pose = '0.2 0 0 0 0 0'
                link_inertia = [link_mass,link_iner,'0','0',link_iner,'0',link_iner]


            subEl = ET.SubElement(root[0],'link',attrib={'name':lnName})
            add_pose2parent(subEl,link_pose,preLink)
            add_inertial2parent(subEl,link_inertia[0],link_inertia[1],link_inertia[2],link_inertia[3],link_inertia[4],link_inertia[5],link_inertia[6])
            add_visual2parent(subEl,visName,vis_pose, 'cylinder',cylinder_len, cylinder_rad)
            if lt == 'body':
                add_collision2parent(subEl,colName,vis_pose, 'cylinder',cylinder_len, cylinder_rad)

            subEl = ET.SubElement(root[0],'joint',attrib={'name':jntName,'type':'revolute'})
            add_pose2parent(subEl,jnt_pose)
            add_tagwithText(subEl,'parent',preLink)
            add_tagwithText(subEl,'child',lnName)
            add_axis2parent(subEl,preLink,jnt_axis)


            if lt == 'body':                

                subEl = ET.SubElement(root[0],'plugin',attrib={'filename':'gz-sim-buoyancy-engine-system','name':'gz::sim::systems::BuoyancyEngine'})
                add_tagwithText(subEl,'link_name',lnName)
                # <namespace>seg_bar</namespace>
                add_tagwithText(subEl,'namespace','cable')
                add_tagwithText(subEl,'min_volume','0.0000')
                add_tagwithText(subEl,'neutral_volume','0.00000001')
                add_tagwithText(subEl,'default_volume','0.00000002')
                add_tagwithText(subEl,'max_volume','0.00000003')
                add_tagwithText(subEl,'max_inflation_rate','0.000000001')
                add_tagwithText(subEl,'surface','0.0')


                subEl = ET.SubElement(root[0],'plugin',attrib={'filename':'gz-sim-hydrodynamics-system','name':'gz::sim::systems::Hydrodynamics'})
                add_tagwithText(subEl,'link_name',lnName)
                add_tagwithText(subEl,'water_density','1000')
                for tg in ['xDotU','yDotV','zDotW','kDotP','mDotQ','nDotR']:
                    add_tagwithText(subEl,tg,'0')
                for tg in ['xU','yV','zW','kP','mQ','nR']:
                    add_tagwithText(subEl,tg,'0')
                add_tagwithText(subEl,'xUabsU','-0.0000005')
                add_tagwithText(subEl,'yVabsV','-0.0000005')
                add_tagwithText(subEl,'zWabsW','-0.0000005')
                add_tagwithText(subEl,'kPabsP','-0.0000001')
                add_tagwithText(subEl,'mQabsQ','-0.0000001')
                add_tagwithText(subEl,'nRabsR','-0.0000001')

            preLink = lnName


        # subEl = ET.SubElement(root[0],'link',attrib={'name':'seg1_pitch'})
        # add_pose2parent(subEl,'0 0 0 0 0 0','cable_base_link')
        # add_inertial2parent(subEl,'1.0e-8','1.0e-7','0','0','1.0e-7','0','1.0e-7')
        # add_visual2parent(subEl,'seg1_pitch_vis','0 0 0 1.5707963267948966  0 0', 'cylinder','0.02','0.01')

        # subEl = ET.SubElement(root[0],'joint',attrib={'name':'seg1_pitch_jnt','type':'revolute'})
        # add_pose2parent(subEl,'0 0 0 0 0 0')
        # add_tagwithText(subEl,'parent','cable_base_link')
        # add_tagwithText(subEl,'child','seg1_pitch')
        # add_axis2parent(subEl,'cable_base_link','0 1 0')



    # file print
    ET.indent(tree, space='\t',level=0)
    tree.write(output_path,encoding="utf-8")


    # print(root)
    # pattern = re.compile(r"@(\w+)")
    # globals()['foo'] will return the value of foo
    # TODO(clyde) trim floats
    # s = re.sub(pattern, lambda m: str(globals()[m.group(1)]), s)
    # open(output_path, "w").write(s)




if __name__ == "__main__":
    # if len(sys.argv) != 4:
    #     print("Usage:")
    #     print("generate_model.py infile outfile 0|1")
    #     print("0: control thrust force")
    #     print("1: control angular velocity")
    #     exit(-100)
    add_cable_seg(sys.argv[1], sys.argv[2])